/**
 * @file main.cpp
 * @author Hedi Basly
 * @brief Entry point for EdgeSense Application
 * @date 2026-02-16
 */
#include <iomanip>
#include <iostream>
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Sensors/LPS25HB_EnvSens.h>
#include <EdgeSense/Sensors/LSM9DS1_ImuSensMag.h>
#include <EdgeSense/Sensors/LSM9DS1_ImuSensAccGyro.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Core/ThreadManager.h>

using namespace EdgeSense::HAL;
using namespace EdgeSense::Sensors;
using namespace EdgeSense::Core;

int main() {
    EdgeSense::Core::ThreadManager manager;
    
    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");

    /* Initialize the I2C Bus on /dev/i2c-1 (Standard RPi 5 pins) */
    I2cMaster i2c("/dev/i2c-1");

    /* Open the Bus */
    if (!i2c.openBus()) {
        LOG_ERROR("CRITICAL: Failed to open I2C bus. Exiting.");
        return -1;
    }

    /* Create Sensor Instances */
    std::unique_ptr <EnvSensors> Pi_LPS25HB = std::make_unique<LPS25HB>(i2c);
    std::unique_ptr <ImuSensors> Pi_LSM9DS1AG = std::make_unique<LSM9DS1_AccGyro>(i2c);
    std::unique_ptr <ImuSensors> Pi_LSM9DS1Mag = std::make_unique<LSM9DS1_Mag>(i2c);
    
    /* Intialize the LPS25HB */
    if (!Pi_LPS25HB->initialize()) {
        LOG_ERROR("Failed to initialize LPS25HB!");
    }
    /* Intialize the LSM9DS1 */
    if (!Pi_LSM9DS1AG->initialize()) {
        LOG_ERROR("Failed to initialize LSM9DS1 Accelerometer/Gyroscope!");
    }
    if (!Pi_LSM9DS1Mag->initialize()) {
        LOG_ERROR("Failed to initialize LSM9DS1 Magnetometer!");
    }
    
    /* --- 1. Harvester Task (1ms) --- */
    manager.setHarvesterTask([&]() {
        /* Capture the raw data from the I2C sensors */
        Pi_LPS25HB->update(); 
        Pi_LSM9DS1AG->update();
        Pi_LSM9DS1Mag->update();

        /* Push the raw Vector3/float data into the Registry's Circular Buffers */
        auto& registry = EdgeSense::Sensors::SensorRegistry::getInstance();
        
        registry.getAccelRawBuffer().push(Pi_LSM9DS1AG->getAcceleration());
        registry.getGyroRawBuffer().push(Pi_LSM9DS1AG->getGyroscope());
        registry.getMagRawBuffer().push(Pi_LSM9DS1Mag->getMagnetometer());
        
        registry.getPressure().push(Pi_LPS25HB->getPressure());
        registry.getTemperature().push(Pi_LPS25HB->getTemperature());
    });

    /* --- 2. Refiner Task (5ms) --- */
    manager.setRefinerTask([&]() {
        auto& registry = EdgeSense::Sensors::SensorRegistry::getInstance();
        
        /* Function to average a window of Vector3 samples */
        auto getAverage = [](const std::vector<EdgeSense::Sensors::Vector3>& samples) {
            EdgeSense::Sensors::Vector3 avg{0, 0, 0};
            if (samples.empty()) return avg;
            for (const auto& s : samples) {
                avg.x += s.x; avg.y += s.y; avg.z += s.z;
            }
            float n = static_cast<float>(samples.size());
            return EdgeSense::Sensors::Vector3{avg.x/n, avg.y/n, avg.z/n};
        };

        /* Process IMU data */
        auto aAvg = getAverage(registry.getAccelRawBuffer().getLatest(5));
        auto gAvg = getAverage(registry.getGyroRawBuffer().getLatest(5));
        auto mAvg = getAverage(registry.getMagRawBuffer().getLatest(5));

        /* Update the Registry's thread-safe "Snapshots" */
        registry.updateFilteredImuAccel(aAvg.x, aAvg.y, aAvg.z);
        registry.updateFilteredImuGyro(gAvg.x, gAvg.y, gAvg.z);
        registry.updateFilteredImuMag(mAvg.x, mAvg.y, mAvg.z);

        /* Process Environmental (take latest 1) */
        auto pLatest = registry.getPressure().getLatest(1);
        auto tLatest = registry.getTemperature().getLatest(1);
        if (!pLatest.empty() && !tLatest.empty()) {
            registry.updateFilteredEnv(pLatest[0], tLatest[0]);
        }
    });

    /* --- 3. Navigator Task (10ms) --- */
    int printDivisor = 0;
    manager.setNavigatorTask([&]() {
        auto& registry = EdgeSense::Sensors::SensorRegistry::getInstance();
        
        float ax, ay, az, gx, gy, gz, mx, my, mz, press, temp;
        
        /* Pull the clean snapshots */
        registry.getFilteredImuAccel(ax, ay, az);
        registry.getFilteredImuGyro(gx, gy, gz);
        registry.getFilteredImuMag(mx, my, mz);
        registry.getFilteredEnv(press, temp);

        /* Print to console at 10Hz (every 100ms) to avoid CPU bloat */
        if (++printDivisor >= 10) {
            std::cout << std::fixed << std::setprecision(2);
        
            /* Dashboard Layout: 
           [IMU] = Motion 
           [MAG] = Heading 
           [ENV] = Atmosphere 
           [JIT] = OS Health 
        */
        std::cout << "\r[IMU] A(" << ax << "," << ay << ") G(" << gx << "," << gy << ") "
                  << "[MAG] M(" << mx << "," << my << "," << mz << ") " /* Now Included */
                  << "[ENV] P:" << press << " T:" << temp << "C "
                  << "| Jitter(H/R/N): " 
                  << (manager.getMaxJitter(EdgeSense::Core::Tier::Harvester) / 1000) << "/"
                  << (manager.getMaxJitter(EdgeSense::Core::Tier::Refiner) / 1000) << "/"
                  << (manager.getMaxJitter(EdgeSense::Core::Tier::Navigator) / 1000) << " us"
                  << std::flush;
            
            printDivisor = 0;
        }
    });

    manager.start();
    
    /* Keep main alive */
    while(true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    
    return 0;
}
