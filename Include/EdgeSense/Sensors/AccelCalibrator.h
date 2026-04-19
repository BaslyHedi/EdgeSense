/**
 * @file AccelCalibrator.h
 * @author Hedi Basly
 * @brief Accelerometer calibration state machine (6-point static capture)
 * @date 2026-04-12
 */

#pragma once

#include <vector>
#include <EdgeSense/Sensors/Sensors.h>
#include "CalibratorBase.h"

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Calibration states for accelerometer (6-position static capture)
         */
        enum class AccelState {
            IDLE,
            INIT_PROMPT,        /* Show instructions */
            WAIT_POSITION_1,    /* Face up (Z+) */
            CAPTURE_POSITION_1,
            WAIT_POSITION_2,    /* Face down (Z-) */
            CAPTURE_POSITION_2,
            WAIT_POSITION_3,    /* Right side (X+) */
            CAPTURE_POSITION_3,
            WAIT_POSITION_4,    /* Left side (X-) */
            CAPTURE_POSITION_4,
            WAIT_POSITION_5,    /* Front side (Y+) */
            CAPTURE_POSITION_5,
            WAIT_POSITION_6,    /* Back side (Y-) */
            CAPTURE_POSITION_6,
            PROCESSING,         /* Computing bias and scale */
            VERIFY,             /* Checking variance thresholds */
            COMPLETE,
            ERROR
        };

        /**
         * @brief Accelerometer calibrator using 6-point static capture method
         * 
         * Captures acceleration data in 6 orthogonal positions (face up/down, 
         * left/right, front/back) and computes bias and scale factors for all three axes.
         */
        class AccelCalibrator : public CalibratorBase {
        public:
            AccelCalibrator();
            virtual ~AccelCalibrator() = default;

            bool startCalibration() override;
            bool processCalibration() override;
            bool isComplete() const override;
            std::string getStateString() const override;

            /**
             * @brief Get computed calibration offsets and scale
             * @param bias Pointer to float[3] to store bias values
             * @param scale Pointer to float[3] to store scale factors
             */
            void getCalibrationData(float* bias, float* scale) const {
                for (int i = 0; i < 3; ++i) {
                    bias[i] = accel_bias[i];
                    scale[i] = accel_scale[i];
                }
            }

            /**
             * @brief Get current calibration state
             * @return Current AccelState
             */
            AccelState getState() const { return state; }

        private:
            AccelState state = AccelState::IDLE;
            std::vector<Vector3> samples;
            float accel_bias[3] = {0};
            float accel_scale[3] = {1.0f, 1.0f, 1.0f};
            int currentPosition = 0;

            /* Constants for calibration */
            const int SAMPLES_PER_POSITION = 200;     /* ~2 seconds at 100Hz */
            const float VARIANCE_THRESHOLD = 0.5f;    /* m/s^2 */

            /* Position instructions for user guidance */
            const char* positionInstructions[6] = {
                "Place device with Z-axis pointing UP (Face Up). Press ENTER to capture.",
                "Place device with Z-axis pointing DOWN (Face Down). Press ENTER to capture.",
                "Place device with X-axis pointing RIGHT. Press ENTER to capture.",
                "Place device with X-axis pointing LEFT. Press ENTER to capture.",
                "Place device with Y-axis pointing FORWARD. Press ENTER to capture.",
                "Place device with Y-axis pointing BACKWARD. Press ENTER to capture."
            };

            /* State machine logic helpers */
            void promptPosition();
            void capturePosition();
            void computeOffsets();
            bool verifyCalibration();
            void transitionToNextPosition();

            /* Position validation and user guidance */
            bool validatePosition(const Vector3& reading);
            void provideOrientationGuidance(const Vector3& reading);
            bool promptRetryOrAbort();
            void displayLiveValues(const Vector3& reading);  /* Live X/Y/Z display during WAIT states */
            bool isEnterPressed();                           /* Non-blocking stdin ENTER check */

            /* Validation state tracking */
            int validationAttempts = 0;
            bool captureStarted = false;  /* Track if we've printed "Position OK" message */
            const int MAX_VALIDATION_ATTEMPTS = 3;
            const float ORIENTATION_THRESHOLD = 0.5f;  /* g units, minimum gravity reading on expected axis (1.0g = 9.81 m/s^2) */
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
