/**
 * @file ImuSensors.cpp
 * @author Hedi Basly
 * @brief Implementation of ImuSensors module
 * @date 2026-02-16
 */

#include "EdgeSense/Sensors/Sensors.h"
#include "EdgeSense/Sensors/LSM9DS1_ImuSens.h"
#include "EdgeSense/Logger/Logger.h"

namespace EdgeSense {
    namespace Sensors {
        LSM9DS1::LSM9DS1(HAL::I2cMaster& bus) : ImuSensor("Accel-Gyro", 0x6A, bus), 
            accel({0.0f, 0.0f, 0.0f}), 
            gyro({0.0f, 0.0f, 0.0f}) {}

        bool LSM9DS1::initialize() {
            uint8_t id = 0;
            bool configresVal = false;
            bool retVal = true;
            
            // 1. Verify WHO_AM_I (Register 0x0F)
            if (!i2cBus.readByte(address, 0x0F, id) || id != 0x68) {
                LOG_ERROR(name + ": ID Mismatch. Expected 0x68, got 0x" + std::to_string(id));
                retVal = false;
            }

            // 2. Global Hardware Config (CTRL_REG8 @ 0x22)
            // 0x44 = [0][1][0][0][0][1][0][0]
            // Bit 6: BDU (Block Data Update) - Wait for all bytes to be read
            // Bit 2: IF_ADD_INC - Auto-increment register address during multi-byte reads
            configresVal = i2cBus.writeByte(address, 0x22, 0x44);

            // 3. Gyroscope Setup (CTRL_REG1_G @ 0x10)
            // 0x60 = 119 Hz ODR, 245 dps Full Scale
            configresVal = i2cBus.writeByte(address, 0x10, 0x60);
            
            // Enable Gyro axes (CTRL_REG4 @ 0x1E)
            configresVal = i2cBus.writeByte(address, 0x1E, 0x38);

            // 4. Accelerometer Setup (CTRL_REG6_XL @ 0x20)
            // 0x60 = 119 Hz ODR, +/- 2g Full Scale
            configresVal = i2cBus.writeByte(address, 0x20, 0x60);

            // Enable Accel axes (CTRL_REG5_XL @ 0x1F)
            configresVal = i2cBus.writeByte(address, 0x1F, 0x38);

            if (!configresVal) {
                LOG_ERROR(name + ": Failed to write configuration.");
                retVal = false;
            }

            LOG_INFO(name + ": Accel/Gyro initialized successfully.");
            return retVal;
        }

        void LSM9DS1::update() {
            uint8_t buf[6];

            // Read Accel (6 bytes starting at 0x28)
            // We use the 0x80 bit for auto-incrementing the register address
            if (i2cBus.readBytes(address, 0x28 | 0x80, buf, 6)) {
                accel.x = stitch(buf[0], buf[1]) * 0.000061f;
                accel.y = stitch(buf[2], buf[3]) * 0.000061f;
                accel.z = stitch(buf[4], buf[5]) * 0.000061f;
            }

            // Read Gyro (6 bytes starting at 0x18)
            if (i2cBus.readBytes(address, 0x18 | 0x80, buf, 6)) {
                gyro.x = stitch(buf[0], buf[1]) * 0.00875f;
                gyro.y = stitch(buf[2], buf[3]) * 0.00875f;
                gyro.z = stitch(buf[4], buf[5]) * 0.00875f;
            }
        }

        int16_t LSM9DS1::stitch(uint8_t low, uint8_t high) const {
            return static_cast<int16_t>((high << 8) | low);
        }

        Vector3 LSM9DS1::getAcceleration() const { return accel; }
        Vector3 LSM9DS1::getGyroscope() const { return gyro; }
        Vector3 LSM9DS1::getMagnetometer() const { return magneto; } 
    } // namespace Sensors
} // namespace EdgeSense
