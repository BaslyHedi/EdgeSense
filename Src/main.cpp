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

using namespace EdgeSense::HAL;
using namespace EdgeSense::Logger;
using namespace EdgeSense::Sensors;

int main() 
{
    /* Start the logger */
    auto& logger = Logger::Logger::getInstance();
    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");

    /* Initialize the I2C Bus on /dev/i2c-1 (Standard RPi 5 pins) */
    I2cMaster i2c("/dev/i2c-1");

    /* Open the Bus */
    if (!i2c.openBus()) {
        LOG_ERROR("CRITICAL: Failed to open I2C bus. Exiting.");
        return -1;
    }

    /* Create Sensors objects */
    std::unique_ptr <EnvSensor> PressureSensor = std::make_unique<LPS25HB>(i2c);

    /* Intialize the hardware */
    if (!PressureSensor->initialize()) {
        LOG_ERROR("Failed to initialize LPS25HB!");
        return -1;
    }
    
    /* Main Telemetry Loop */
    try {
        while (true) {
            /* Pull new data from the physical registers */
            PressureSensor->update();

            /* Format the output nicely */
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2)
               << " [ " << PressureSensor->getName() << " ] "
               << " Pressure: " << PressureSensor->getPressure() << " hPa | "
               << " Temp: " << PressureSensor->getTemperature() << " °C";

            std::cout << ss.str() << std::endl;
            

            /* Wait 1 second before next reading */
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    } catch (...) {
        LOG_WARN("Loop interrupted.");
    }
    
    /* Close I2C bus and Flush logger */
    i2c.closeBus();
    logger.stop();
    return 0;
}
