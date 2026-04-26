/**
 * @file LSM9DS1_ImuSensMag.cpp
 * @author Hedi Basly
 * @brief Implementation of LSM9DS1 ImuSensor module for Magnetometer
  * This module provides the implementation for interfacing with the LSM9DS1 IMU sensor, specifically
  * for reading magnetometer data. It includes:
  * - Initialization of the sensor with appropriate configuration settings for the magnetometer.
  * - An update method that reads raw data from the sensor registers, converts it to physical units (Ga), and stores it in a member variable.
  * - A helper method `stitch` to combine high and low bytes into a signed 16-bit integer, which is necessary for interpreting the raw sensor data correctly.
 * @date 2026-02-16
 */

#include "EdgeSense/Sensors/Sensors.h"
#include "EdgeSense/Sensors/LSM9DS1_ImuSensMag.h"
#include "EdgeSense/Logger/Logger.h"

namespace EdgeSense {
    namespace Sensors {
        LSM9DS1_Mag::LSM9DS1_Mag(HAL::I2cMaster& bus) : ImuSensors(LSM9DS1_IMU_MAG_NAME, 0x1C, bus), 
            magneto({0.0f, 0.0f, 0.0f}) {}

        bool LSM9DS1_Mag::initialize() {
            uint8_t id = 0;
            bool configresVal = true;
            bool retVal = true;
            
            /* 1. Verify WHO_AM_I (Register 0x0F) */
            if (!i2cBus.readByte(address, 0x0F, id) || id != 0x3D) {
                LOG_ERROR(name + ": ID Mismatch. Expected 0x3D, got 0x" + std::to_string(id));
                retVal = false;
            }

            /* 2. Set X/Y to Ultra-High Performance and ODR to 80Hz */
            /* 0x7C = 0111 1100 in binary */
            configresVal &= i2cBus.writeByte(address, 0x20, 0x7C);

            /* 3. Set Full Scale to +/- 4 Gauss */
            configresVal &= i2cBus.writeByte(address, 0x21, 0x00);

            /* 4. Enable Continuous Conversion Mode (Wake up!) */
            configresVal &= i2cBus.writeByte(address, 0x22, 0x00);

            /* 5. Set Z to Ultra-High Performance */
            configresVal &= i2cBus.writeByte(address, 0x23, 0x0C);

            /* 6. Block Data Update (BDU) */
            /* Register 0x24 bit 6 ensures we don't read MSB and LSB from different samples */
            configresVal &= i2cBus.writeByte(address, 0x24, 0x40);

            if (!configresVal) {
                LOG_ERROR(name + ": Failed to write configuration.");
                retVal = false;
            }

            LOG_INFO(name + ": initialized successfully.");
            return retVal;
        }

        void LSM9DS1_Mag::update() {
            uint8_t buf[6];

            /* Read Mag (6 bytes starting at 0x28) */
            /* We use the 0x80 bit for auto-incrementing the register address */
            if (i2cBus.readBytes(address, 0x28 | 0x80, buf, 6)) {
                /* Sensitivity for +/- 4 Gauss is 0.14 mG/LSB */
                magneto.x = stitch(buf[0], buf[1]) * 0.00014f;
                magneto.y = stitch(buf[2], buf[3]) * 0.00014f;
                magneto.z = stitch(buf[4], buf[5]) * 0.00014f;
            } else {
                LOG_ERROR(name + ": Failed to read magnetometer data.");
            }
        }

        int16_t LSM9DS1_Mag::stitch(uint8_t low, uint8_t high) const {
            return static_cast<int16_t>((high << 8) | low);
        }

        Vector3 LSM9DS1_Mag::getMagnetometer() const { return magneto; } 
    } /* namespace Sensors */
} /* namespace EdgeSense */
