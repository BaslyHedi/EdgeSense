/**
 * @file SensorManager.cpp
 * @author Hedi Basly
 * @brief Implementation of SensorManager module 
 * @date 2026-04-06
 */
#include <EdgeSense/Core/SensorManager.h>
#include <EdgeSense/Logger/Logger.h>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace EdgeSense::Core;
using namespace EdgeSense::Sensors;
using namespace EdgeSense::Logger;
using namespace EdgeSense::HAL;

namespace EdgeSense {
    namespace Core {

    SensorManager::SensorManager(ThreadManager& tm)
        : tm(tm), I2c("/dev/i2c-1") 
        {
            /* Create the calibration engine insatnce to initilize the data */
            CalibrationEngine::getInstance();
        }

    bool SensorManager::init() {
        bool retVal = true;

        /* Open the Bus */
        if (!I2c.openBus()) {
            LOG_ERROR("CRITICAL: Failed to open I2C bus. Exiting.");
            retVal = false;
        }
        /* Intialize the LPS25HB */
        if (!Pi_LPS25HB->initialize()) {
            LOG_ERROR("Failed to initialize LPS25HB!");
            retVal = false;
        }
        /* Intialize the LSM9DS1 */
        if (!Pi_LSM9DS1AG->initialize()) {
            LOG_ERROR("Failed to initialize LSM9DS1 Accelerometer/Gyroscope!");
            retVal = false;
        }
        if (!Pi_LSM9DS1Mag->initialize()) {
            LOG_ERROR("Failed to initialize LSM9DS1 Magnetometer!");
            retVal = false;
        }

        this->setupAppTasks();
        this->setupCalibTasks();

        return retVal;
    }

    void SensorManager::calibHarvestAction() {
        /* Harvest raw samples from sensors */
        Pi_LSM9DS1AG->update();
        Pi_LSM9DS1Mag->update();
        Pi_LPS25HB->update();

        /* Push raw data into registry so calibration engine can access it */
        auto& registry = EdgeSense::Sensors::SensorsRegistry::getInstance();

        registry.getAccelRawBuffer().push(Pi_LSM9DS1AG->getAcceleration());
        registry.getGyroRawBuffer().push(Pi_LSM9DS1AG->getGyroscope());
        registry.getMagRawBuffer().push(Pi_LSM9DS1Mag->getMagnetometer());

        registry.getPressure().push(Pi_LPS25HB->getPressure());
        registry.getTemperature().push(Pi_LPS25HB->getTemperature());

        /* CalibrationEngine will collect samples from registry via harvestRawSamples() */
        auto& engine = EdgeSense::Core::CalibrationEngine::getInstance();
        engine.harvestRawSamples();
    }

    void SensorManager::calibProcessAction() {
        /* Process calibration state machine at 100Hz */
        auto& engine = EdgeSense::Core::CalibrationEngine::getInstance();
        engine.processCalibration();
    }

    void SensorManager::appHarvestAction() {
        /* * This runs at 1000Hz. 
         * It captures the raw data from the I2C sensors and pushes it into the registry.
         */
        Pi_LSM9DS1AG->update();
        Pi_LSM9DS1Mag->update();
        Pi_LPS25HB->update();

        /* Push the raw Vector3/float data into the Registry's Circular Buffers */
        auto& registry = EdgeSense::Sensors::SensorsRegistry::getInstance();

        registry.getAccelRawBuffer().push(Pi_LSM9DS1AG->getAcceleration());
        registry.getGyroRawBuffer().push(Pi_LSM9DS1AG->getGyroscope());
        registry.getMagRawBuffer().push(Pi_LSM9DS1Mag->getMagnetometer());

        registry.getPressure().push(Pi_LPS25HB->getPressure());
        registry.getTemperature().push(Pi_LPS25HB->getTemperature());
    }

    void SensorManager::appRefineAction() {
        /* * This runs at 200Hz. 
         * It takes the raw data, applies the calibration offsets, 
         * and saves the clean data back to the registry.
         */
        auto& registry = EdgeSense::Sensors::SensorsRegistry::getInstance();
        
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

        /* Apply calibration offsets */
        CalibrationEngine::getInstance().applyCalibrationOffsets(aAvg, gAvg, mAvg);

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
    }

    void SensorManager::appProcessAction() {
        /* * This runs at 100Hz. 
         * Handles logging, IPC broadcast, or AHRS math.
         */
        static int printDivisor = 0;
        auto& registry = EdgeSense::Sensors::SensorsRegistry::getInstance();
        
        float ax, ay, az, gx, gy, gz, mx, my, mz, press, temp;
        
        /* Pull the clean snapshots */
        registry.getFilteredImuAccel(ax, ay, az);
        registry.getFilteredImuGyro(gx, gy, gz);
        registry.getFilteredImuMag(mx, my, mz);
        registry.getFilteredEnv(press, temp);

        /* Log sensor data at 10Hz (every 100ms) to avoid CPU bloat */
        if (++printDivisor >= 10) {
            /* Dashboard: [IMU]=Motion [MAG]=Heading [ENV]=Atmosphere [JIT]=OS Health */
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "[IMU] Accel: (" << ax << ", " << ay << ", " << az << ")"
                << " | Gyro: (" << gx << ", " << gy << ", " << gz << ")"
                << " | [MAG] (" << mx << ", " << my << ", " << mz << ")"
                << " | [ENV] " << press << "hPa " << temp << "C"
                << " | [JIT] H:" << (tm.getMaxJitter(EdgeSense::Core::Tier::HARVESTER) / 1000)
                << "us R:" << (tm.getMaxJitter(EdgeSense::Core::Tier::REFINER) / 1000)
                << "us P:" << (tm.getMaxJitter(EdgeSense::Core::Tier::PROCESS) / 1000) << "us";
            LOG_INFO(oss.str());
            printDivisor = 0;
        }
    }

    void SensorManager::setupAppTasks() {
        /* Notice how clean this looks now! The ThreadManager just calls our actions. */
        tm.setHarvesterTask(ExecutionMode::APP, [this]() { appHarvestAction(); });
        tm.setRefinerTask(ExecutionMode::APP,   [this]() { appRefineAction(); });
        tm.setProcessTask(ExecutionMode::APP,   [this]() { appProcessAction(); });
    }

    void SensorManager::setupCalibTasks() {
        /* Calibration specific logic */
        tm.setHarvesterTask(ExecutionMode::CALIB, [this]() { calibHarvestAction(); });
        tm.setProcessTask(ExecutionMode::CALIB,   [this]() { calibProcessAction(); });
    }


    void SensorManager::runApplication() {
        tm.stop(); /* Ensure clean slate */
        tm.start();
        tm.setExecutionMode(ExecutionMode::APP);
    }

    void SensorManager::runCalibration() {
        LOG_INFO("Starting calibration mode...");

        tm.stop(); /* Ensure clean slate */
        tm.start();
        tm.setExecutionMode(ExecutionMode::CALIB);

        /* Initialize and start the calibration sequence */
        auto& engine = EdgeSense::Core::CalibrationEngine::getInstance();
        if (!engine.startCalibrationSequence()) {
            LOG_ERROR("Failed to start calibration sequence");
            tm.setExecutionMode(ExecutionMode::APP);
        } else {
            /* Wait for calibration to complete */
            while (!engine.isCalibrationComplete()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            LOG_INFO("Calibration sequence complete");

            /* Save calibration data */
            engine.saveAllCalibrationData();

            /* Return to application mode */
            LOG_INFO("Returning to application mode");
            tm.setExecutionMode(ExecutionMode::APP);
        }
    }

    } /* namespace Core */
} /* namespace EdgeSense */
