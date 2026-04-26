/**
 * @file LSM9DS1_ImuSensAccGyro.cpp
 * @author Hedi Basly
 * @brief Implementation of LSM9DS1 ImuSensor module for Accelerometer and Gyroscope
  * This module provides the implementation for interfacing with the LSM9DS1 IMU sensor, specifically
  * for reading acceleration and gyroscope data. It includes:
  * - Initialization of the sensor with appropriate configuration settings for both accelerometer and gyroscope.
  * - An update method that reads raw data from the sensor registers, converts it to physical units (g for accel, dps for gyro), and stores it in member variables.
  * - A helper method `stitch` to combine high and low bytes into a signed 16-bit integer, which is necessary for interpreting the raw sensor data correctly.
 * @date 2026-02-16
 */

#include "EdgeSense/Sensors/Sensors.h"
#include "EdgeSense/Sensors/LSM9DS1_ImuSensAccGyro.h"
#include "EdgeSense/Logger/Logger.h"

namespace EdgeSense {
    namespace Sensors {
        LSM9DS1_AccGyro::LSM9DS1_AccGyro(HAL::I2cMaster& bus) : ImuSensors(LSM9DS1_IMU_ACCGYRO_NAME, 0x6A, bus),
            accel({0.0f, 0.0f, 0.0f}),
            gyro({0.0f, 0.0f, 0.0f}) {}

        bool LSM9DS1_AccGyro::initialize() {
            uint8_t id = 0;
            bool configresVal = true;
            bool retVal = true;

            /* 1. Verify WHO_AM_I (Register 0x0F) */
            if (!i2cBus.readByte(address, 0x0F, id) || id != 0x68) {
                LOG_ERROR(name + ": ID Mismatch. Expected 0x68, got 0x" + std::to_string(id));
                retVal = false;
            }

            /* 2. Global Hardware Config (CTRL_REG8 @ 0x22) */
            /* 0x44 = [0][1][0][0][0][1][0][0]                */
            /* Bit 6: BDU (Block Data Update) - wait for all bytes to be read */
            /* Bit 2: IF_ADD_INC - auto-increment register address during multi-byte reads */
            configresVal &= i2cBus.writeByte(address, 0x22, 0x44);

            /* 3. Gyroscope Setup (CTRL_REG1_G @ 0x10) */
            /* 0x60 = 119 Hz ODR, 245 dps Full Scale */
            configresVal &= i2cBus.writeByte(address, 0x10, 0x60);

            /* Enable Gyro axes (CTRL_REG4 @ 0x1E) */
            configresVal &= i2cBus.writeByte(address, 0x1E, 0x38);

            /* 4. Accelerometer Setup (CTRL_REG6_XL @ 0x20) */
            /* 0x60 = 119 Hz ODR, +/- 2g Full Scale */
            configresVal &= i2cBus.writeByte(address, 0x20, 0x60);

            /* Enable Accel axes (CTRL_REG5_XL @ 0x1F) */
            configresVal &= i2cBus.writeByte(address, 0x1F, 0x38);

            if (!configresVal) {
                LOG_ERROR(name + ": Failed to write configuration.");
                retVal = false;
            }

            LOG_INFO(name + ": initialized successfully.");
            return retVal;
        }

        void LSM9DS1_AccGyro::update() {
            uint8_t buf[6];

            /* Read Accel (6 bytes starting at 0x28) */
            /* Use 0x80 bit for auto-incrementing the register address */
            if (i2cBus.readBytes(address, 0x28 | 0x80, buf, 6)) {
                accel.x = stitch(buf[0], buf[1]) * 0.000061f;
                accel.y = stitch(buf[2], buf[3]) * 0.000061f;
                accel.z = stitch(buf[4], buf[5]) * 0.000061f;
            }

            /* Read Gyro (6 bytes starting at 0x18) */
            if (i2cBus.readBytes(address, 0x18 | 0x80, buf, 6)) {
                gyro.x = stitch(buf[0], buf[1]) * 0.00875f;
                gyro.y = stitch(buf[2], buf[3]) * 0.00875f;
                gyro.z = stitch(buf[4], buf[5]) * 0.00875f;
            }
        }

        int16_t LSM9DS1_AccGyro::stitch(uint8_t low, uint8_t high) const {
            return static_cast<int16_t>((high << 8) | low);
        }

        Vector3 LSM9DS1_AccGyro::getAcceleration() const { return accel; }
        Vector3 LSM9DS1_AccGyro::getGyroscope() const { return gyro; }
    } /* namespace Sensors */
} /* namespace EdgeSense */
