/**
 * @file PressureTempCalibrator.h
 * @author Hedi Basly
 * @brief Pressure and temperature calibration (simplified baseline capture)
 * @date 2026-04-12
 */

#pragma once

#include "CalibratorBase.h"

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Environmental sensors calibration (simplified - no complex state machine)
         * 
         * Pressure and temperature sensors typically don't require complex calibration.
         * This calibrator captures baseline values during a stationary period for reference.
         */
        class PressureTempCalibrator : public CalibratorBase {
        public:
            PressureTempCalibrator();
            virtual ~PressureTempCalibrator() = default;

            bool startCalibration() override;
            bool processCalibration() override;
            bool isComplete() const override;
            std::string getStateString() const override;

            /**
             * @brief Get baseline pressure and temperature readings
             * @param baseline_pressure Pointer to float for baseline pressure (hPa)
             * @param baseline_temp Pointer to float for baseline temperature (°C)
             */
            void getCalibrationData(float* baseline_pressure, float* baseline_temp) const {
                *baseline_pressure = baseline_press;
                *baseline_temp = baseline_temperature;
            }

        private:
            float baseline_press = 0;
            float baseline_temperature = 0;
            bool calibrationDone = false;

            /* Constants for calibration */
            const int BASELINE_SAMPLES = 200;   /* ~2 seconds at 100Hz */

            /* State machine logic helper */
            void captureBaseline();
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
