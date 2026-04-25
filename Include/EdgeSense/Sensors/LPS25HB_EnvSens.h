/**
 * @file LPS25HB_EnvSens.h
 * @author Hedi Basly
 * @brief Header for LPS25HB Environmental Sensors module
 * @date 2026-02-16
 */

#pragma once

#include "EdgeSense/Sensors/Sensors.h"

namespace EdgeSense {
    namespace Sensors {
        class LPS25HB : public EnvSensors {
        public:
            LPS25HB(HAL::I2cMaster& bus);
            
            bool initialize() override;
            void update() override;
            float getTemperature() const override;
            float getPressure() const override;

        private:
            float temp;
            float pressure;
        };
    } /* namespace Sensors */
} /* namespace EdgeSense */
