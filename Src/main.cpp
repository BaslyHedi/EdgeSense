/**
 * @file main.cpp
 * @author Hedi Basly
 * @brief Entry point for EdgeSense Application
 * @date 2026-02-16
 */
#include <iostream>
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/HAL/I2cMaster.h>

using namespace EdgeSense;
using namespace HAL;

int main() 
{
    auto& logger = Logger::Logger::getInstance();
    // 1. Initialize the I2C Bus on /dev/i2c-1 (Standard RPi 5 pins)
    I2cMaster i2c("/dev/i2c-1");
    
    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");

    // 2. Open the Bus
    if (!i2c.openBus()) {
        LOG_ERROR("CRITICAL: Failed to open I2C bus. Check permissions or raspi-config!");
        return -1;
    }

    // 3. Test: Read "Who Am I" from LPS25HB (Pressure Sensor @ 0x5C)
    // Register 0x0F is the standard 'Who Am I' for STMicroelectronics sensors
    uint8_t pressureID = 0;
    const uint8_t LPS25HB_ADDR = 0x5C;
    const uint8_t WAI_REG = 0x0F;

    if (i2c.readByte(LPS25HB_ADDR, WAI_REG, pressureID)) {
        std::stringstream ss;
        ss << "✅ LPS25HB Found! Hardware ID: 0x" << std::hex << (int)pressureID 
           << " (Expected: 0xbd)";
        LOG_INFO(ss.str());
    } else {
        LOG_ERROR("❌ LPS25HB not responding. Check wiring or address 0x5C.");
    }

    // 4. Test: Read "Who Am I" from HTS221 (Humidity Sensor @ 0x5F)
    uint8_t humidityID = 0;
    const uint8_t HTS221_ADDR = 0x5F;

    if (i2c.readByte(HTS221_ADDR, WAI_REG, humidityID)) {
        std::stringstream ss;
        ss << "✅ HTS221 Found! Hardware ID: 0x" << std::hex << (int)humidityID 
           << " (Expected: 0xbc)";
        LOG_INFO(ss.str());
    } else {
        LOG_WARN("⚠️ HTS221 not detected at 0x5F.");
    }

    // 5. Cleanup and Shutdown
    LOG_INFO("🏁 Hardware discovery complete. Shutting down.");
    i2c.closeBus();
    
    // Final flush of the logger
    logger.stop();
    return 0;
}
