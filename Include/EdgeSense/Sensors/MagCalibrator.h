/**
 * @file MagCalibrator.h
 * @author Hedi Basly
 * @brief Magnetometer calibration state machine (3D rotation capture)
 * @date 2026-04-12
 */

#pragma once

#include <vector>
#include <array>
#include <EdgeSense/Sensors/Sensors.h>
#include "CalibratorBase.h"

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Calibration states for magnetometer (3D rotation capture)
         */
        enum class MagState {
            IDLE,
            INIT_PROMPT,        /* Show instructions */
            WAIT_ROTATION,      /* User ready for rotation sequence */
            CAPTURE_ROTATION,   /* Collect data during 3D rotations */
            PROCESSING,         /* Computing hard-iron and soft-iron corrections */
            VERIFY,             /* Checking data quality */
            COMPUTE_SCALE,      /* Ellipsoid fitting for soft-iron calibration */
            COMPLETE,
            ERROR
        };

        /**
         * @brief Magnetometer calibrator using 3D rotation capture method
         * 
         * Captures magnetometer data while rotating the device in all three dimensions.
         * Computes both hard-iron offsets (translation) and soft-iron scale factors (ellipsoid fitting).
         */
        class MagCalibrator : public CalibratorBase {
        public:
            MagCalibrator();
            virtual ~MagCalibrator() = default;

            bool startCalibration() override;
            bool processCalibration() override;
            bool isComplete() const override;
            std::string getStateString() const override;

            /**
             * @brief Get computed calibration offsets and scale factors
             * @param bias Pointer to float[3] to store hard-iron bias
             * @param scale Pointer to float[3] to store soft-iron scale factors
             */
            void getCalibrationData(float* bias, float* scale) const {
                for (int i = 0; i < 3; ++i) {
                    bias[i] = mag_bias[i];
                    scale[i] = mag_scale[i];
                }
            }

            /**
             * @brief Get current calibration state
             * @return Current MagState
             */
            MagState getState() const { return state; }

        private:
            MagState state = MagState::IDLE;
            std::vector<Vector3> samples;
            float mag_bias[3] = {0};
            float mag_scale[3] = {1.0f, 1.0f, 1.0f};

            /* Constants for calibration */
            const int CAPTURE_SAMPLES = 800;              /* ~8 seconds at 100Hz */
            const float VARIANCE_THRESHOLD = 50.0f;      /* µT (microtesla) */

            /* State machine logic helpers */
            void promptRotationSequence();
            void captureRotationSamples();
            void computeOffsets();                        /* Hard-iron (min-max method or least-squares) */
            void computeScale();                          /* Soft-iron (ellipsoid fitting) */
            bool verifyCalibration();
            
            /* Helper for ellipsoid fitting */
            void fitEllipsoid();
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
