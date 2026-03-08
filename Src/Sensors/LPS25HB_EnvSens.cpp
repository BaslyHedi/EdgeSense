/**
 * @file LPS25HB_EnvSens.cpp
 * @author Hedi Basly
 * @brief Implementation of LPS25HB_EnvSens module
 * @date 2026-02-16
 */

#include "EdgeSense/Sensors/Sensors.h"
#include "EdgeSense/Sensors/LPS25HB_EnvSens.h"
#include "EdgeSense/Logger/Logger.h"

namespace EdgeSense {
    namespace Sensors {
        LPS25HB::LPS25HB(HAL::I2cMaster& bus) 
        : EnvSensor("LPS25HB", 0x5C, bus), temp(0.0f), pressure(0.0f) {}

        bool LPS25HB::initialize() {
            uint8_t id = 0;
            bool configresVal = false;
            bool retVal = true;
            
            // 1. Verify "Who Am I" (Register 0x0F)
            if (!i2cBus.readByte(address, 0x0F, id) || id != 0xBD) {
                LOG_ERROR(name + ": Hardware ID Mismatch! Expected 0xBD, got 0x" + std::to_string(id));
                retVal = false;
            }

            // --- CTRL_REG1 (0x20) ---
            // Value: 0x94 -> 1 [Active] 001 [1Hz ODR] 0 [Reset Off] 1 [BDU On] 00
            // Setting BDU (Bit 2) is a "Senior Move" for stability.
            configresVal = i2cBus.writeByte(address, 0x20, 0x94);

            // --- RES_CONF (0x10) ---
            // Value: 0x0F -> Internal average configuration.
            // This performs hardware-level averaging to smooth out the pressure noise.
            configresVal = i2cBus.writeByte(address, 0x10, 0x0F);

            if (!configresVal) {
                LOG_ERROR(name + ": Failed to write configuration.");
                retVal = false;
            }

            LOG_INFO(name + ": Initialized successfully at 0x5C.");
            return retVal;
        }

        void LPS25HB::update() {
            // 1. Read Pressure (24-bit: XL, L, H starting at 0x28)
            // We use address | 0x80 to enable the auto-increment feature of the sensor
            uint8_t pBuffer[3];
            if (i2cBus.readBytes(address, 0x28 | 0x80, pBuffer, 3)) {
                // Stitch 3 bytes into a 32-bit signed integer
                int32_t rawP = (static_cast<int32_t>(pBuffer[2]) << 16) | 
                            (static_cast<int32_t>(pBuffer[1]) << 8) | 
                                pBuffer[0];
                
                // Convert to hPa (Divide by 4096 per datasheet)
                pressure = static_cast<float>(rawP) / 4096.0f;
            }

            // 2. Read Temperature (16-bit: L, H starting at 0x2B)
            uint8_t tBuffer[2];
            if (i2cBus.readBytes(address, 0x2B | 0x80, tBuffer, 2)) {
                int16_t rawT = static_cast<int16_t>((tBuffer[1] << 8) | tBuffer[0]);
                
                // Convert to Celsius: Temp = 42.5 + (raw / 480)
                temp = 42.5f + (static_cast<float>(rawT) / 480.0f);
            }
        }

        float LPS25HB::getTemperature() const { return temp; }
        float LPS25HB::getPressure() const { return pressure; }
    }
}
