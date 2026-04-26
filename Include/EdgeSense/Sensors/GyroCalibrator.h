/**
 * @file GyroCalibrator.h
 * @author Hedi Basly
 * @brief Gyroscope calibration state machine (static at-rest capture)
 * @date 2026-04-12
 */

#pragma once

#include <vector>
#include <EdgeSense/Sensors/Sensors.h>
#include "CalibratorBase.h"

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Calibration states for gyroscope (static at-rest capture)
         */
        enum class GyroState {
            IDLE,
            INIT_PROMPT,        /* Show instructions */
            WAIT_STATIC,        /* User ready device on level surface */
            CAPTURE_STATIC,     /* Collect data at rest */
            PROCESSING,         /* Computing bias (gyro only needs bias) */
            VERIFY,             /* Checking variance threshold */
            COMPLETE,
            ERROR
        };

        /**
         * @brief Gyroscope calibrator using static at-rest capture method
         * 
         * Captures gyroscope data while the device is stationary on a level surface.
         * Gyroscopes only require bias calibration (no scale needed for rate data).
         */
        class GyroCalibrator : public CalibratorBase {
        public:
            GyroCalibrator();
            virtual ~GyroCalibrator() = default;

            bool startCalibration() override;
            bool processCalibration() override;
            bool isComplete() const override;
            bool isError() const override { return state == GyroState::ERROR; }
            std::string getStateString() const override;

            /**
             * @brief Get computed calibration bias
             * Gyroscopes only need bias correction, not scale
             * @param bias Pointer to float[3] to store bias values
             */
            void getCalibrationData(float* bias) const {
                for (int i = 0; i < 3; ++i) {
                    bias[i] = gyro_bias[i];
                }
            }

            /**
             * @brief Get current calibration state
             * @return Current GyroState
             */
            GyroState getState() const { return state; }

        private:
            GyroState state = GyroState::IDLE;
            std::vector<Vector3> samples;
            float gyro_bias[3] = {0};

            /* Constants for calibration */
            const int CAPTURE_SAMPLES = 500;              /* ~5 seconds at 100Hz */
            const float VARIANCE_THRESHOLD = 0.05f;      /* dps^2 - relaxed for fan vibration */

            /* State machine logic helpers */
            void promptStaticPosition();
            void captureStaticSamples();
            void computeOffsets();
            bool verifyCalibration();
            void displayLiveValues(const Vector3& reading);
            bool isEnterPressed();
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
