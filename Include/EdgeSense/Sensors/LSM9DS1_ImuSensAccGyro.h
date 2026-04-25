/**
 * @file LSM9DS1_ImuSensAccGyro.h
 * @author Hedi Basly
 * @brief Header for LSM9DS1 ImuSensor module for Accelerometer and Gyroscope
 * @date 2026-02-16
 */
#pragma once

#include "EdgeSense/Sensors/Sensors.h"

namespace EdgeSense {
    namespace Sensors {

        class LSM9DS1_AccGyro : public ImuSensors {
        public:
            LSM9DS1_AccGyro(HAL::I2cMaster& bus);
            
            bool initialize() override;
            void update() override;
            Vector3 getAcceleration() const override;
            Vector3 getGyroscope() const override;

        private:
            Vector3 accel;
            Vector3 gyro;
            Vector3 magneto;

            /* Helper to combine high/low bytes into a signed 16-bit integer */
            int16_t stitch(uint8_t low, uint8_t high) const;
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
