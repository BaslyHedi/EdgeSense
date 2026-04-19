/**
 * @file AccelCalibrator.cpp
 * @author Hedi Basly
 * @brief Implementation of accelerometer calibration via 6-point static capture
 * @date 2026-04-19
 */

#include <EdgeSense/Sensors/AccelCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace EdgeSense::Logger;

namespace EdgeSense {
    namespace Sensors {

        AccelCalibrator::AccelCalibrator() 
            : state(AccelState::IDLE), currentPosition(0) {
        }

        bool AccelCalibrator::startCalibration() {
            state = AccelState::INIT_PROMPT;
            samples.clear();
            currentPosition = 0;
            collectedSamples = 0;
            targetSampleCount = SAMPLES_PER_POSITION * 6;
            validationAttempts = 0;

            LOG_INFO("Accelerometer calibration starting. Total samples target: " + std::to_string(targetSampleCount));
            promptPosition();
            return true;
        }

        bool AccelCalibrator::processCalibration() {
            if (state == AccelState::IDLE || state == AccelState::COMPLETE || state == AccelState::ERROR) {
                return false;
            }

            auto& registry = SensorsRegistry::getInstance();

            switch (state) {
                case AccelState::INIT_PROMPT:
                    state = AccelState::WAIT_POSITION_1;
                    break;

                case AccelState::WAIT_POSITION_1:
                case AccelState::WAIT_POSITION_2:
                case AccelState::WAIT_POSITION_3:
                case AccelState::WAIT_POSITION_4:
                case AccelState::WAIT_POSITION_5:
                case AccelState::WAIT_POSITION_6:
                    /* User should have confirmed - transition to capture */
                    state = static_cast<AccelState>(static_cast<int>(state) + 1);
                    capturePosition();
                    break;

                case AccelState::CAPTURE_POSITION_1:
                case AccelState::CAPTURE_POSITION_2:
                case AccelState::CAPTURE_POSITION_3:
                case AccelState::CAPTURE_POSITION_4:
                case AccelState::CAPTURE_POSITION_5:
                case AccelState::CAPTURE_POSITION_6: {
                    /* Collect samples from registry */
                    auto latestSamples = registry.getAccelRawBuffer().getLatest(1);
                    if (!latestSamples.empty()) {
                        Vector3 reading = latestSamples[0];

                        /* Validate position for first few samples (50 samples = ~0.5 sec) */
                        int positionSampleCount = collectedSamples % SAMPLES_PER_POSITION;
                        if (positionSampleCount < 50) {
                            if (!validatePosition(reading)) {
                                /* Position invalid - reject this capture and re-prompt */
                                if (validationAttempts < MAX_VALIDATION_ATTEMPTS) {
                                    validationAttempts++;
                                    LOG_WARN("Position " + std::to_string(currentPosition + 1) + " validation failed (attempt " 
                                        + std::to_string(validationAttempts) + "/" + std::to_string(MAX_VALIDATION_ATTEMPTS) + ")");
                                    provideOrientationGuidance(reading);
                                    std::cout << "\nPress ENTER after correcting the position...\n";
                                    std::cin.ignore();
                                    validationAttempts = 0;  /* Reset for next attempt */
                                } else {
                                    LOG_ERROR("Position " + std::to_string(currentPosition + 1) + " validation failed after " 
                                        + std::to_string(MAX_VALIDATION_ATTEMPTS) + " attempts");
                                    state = AccelState::ERROR;
                                    return false;
                                }
                                break;  /* Skip this sample and wait for correction */
                            } else {
                                validationAttempts = 0;  /* Reset counter once valid */
                            }
                        }

                        samples.push_back(reading);
                        collectedSamples++;
                    }

                    /* Check if we have enough samples for this position */
                    if (collectedSamples % SAMPLES_PER_POSITION == 0) {
                        currentPosition++;
                        if (currentPosition < 6) {
                            /* Move to next position */
                            state = static_cast<AccelState>(static_cast<int>(AccelState::WAIT_POSITION_1) + (currentPosition * 2));
                            validationAttempts = 0;  /* Reset for new position */
                            promptPosition();
                        } else {
                            /* All positions captured - process */
                            state = AccelState::PROCESSING;
                            computeOffsets();
                        }
                    }
                    break;
                }

                case AccelState::PROCESSING:
                    state = AccelState::VERIFY;
                    if (verifyCalibration()) {
                        state = AccelState::COMPLETE;
                        LOG_INFO("Accelerometer calibration COMPLETE");
                    } else {
                        LOG_ERROR("Accelerometer calibration verification failed");
                        state = AccelState::ERROR;
                    }
                    break;

                default:
                    break;
            }

            return true;
        }

        bool AccelCalibrator::isComplete() const {
            return state == AccelState::COMPLETE;
        }

        std::string AccelCalibrator::getStateString() const {
            switch (state) {
                case AccelState::IDLE:
                    return "IDLE";
                case AccelState::INIT_PROMPT:
                    return "INITIALIZING";
                case AccelState::WAIT_POSITION_1:
                case AccelState::WAIT_POSITION_2:
                case AccelState::WAIT_POSITION_3:
                case AccelState::WAIT_POSITION_4:
                case AccelState::WAIT_POSITION_5:
                case AccelState::WAIT_POSITION_6:
                    return "WAITING_POSITION_" + std::to_string((static_cast<int>(state) - static_cast<int>(AccelState::WAIT_POSITION_1)) / 2 + 1);
                case AccelState::CAPTURE_POSITION_1:
                case AccelState::CAPTURE_POSITION_2:
                case AccelState::CAPTURE_POSITION_3:
                case AccelState::CAPTURE_POSITION_4:
                case AccelState::CAPTURE_POSITION_5:
                case AccelState::CAPTURE_POSITION_6:
                    return "CAPTURING_POSITION_" + std::to_string((static_cast<int>(state) - static_cast<int>(AccelState::CAPTURE_POSITION_1)) / 2 + 1);
                case AccelState::PROCESSING:
                    return "PROCESSING_OFFSETS";
                case AccelState::VERIFY:
                    return "VERIFYING";
                case AccelState::COMPLETE:
                    return "COMPLETE";
                case AccelState::ERROR:
                    return "ERROR";
                default:
                    return "UNKNOWN";
            }
        }

        void AccelCalibrator::promptPosition() {
            std::cout << "\n" << std::string(50, '-') << "\n";
            std::cout << "Position " << (currentPosition + 1) << " of 6\n";
            std::cout << positionInstructions[currentPosition] << "\n";
            std::cout << std::string(50, '-') << "\n";
            std::cout << "Press ENTER when ready...\n";
            std::cin.ignore();
        }

        void AccelCalibrator::capturePosition() {
            LOG_INFO("Starting capture for position " + std::to_string(currentPosition + 1) + "/6");
        }

        void AccelCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) < SAMPLES_PER_POSITION * 6) {
                LOG_ERROR("Insufficient samples for accelerometer calibration");
                return;
            }

            /* Calculate average for each position */
            Vector3 positionAverages[6];

            for (int pos = 0; pos < 6; ++pos) {
                Vector3 sum = {0, 0, 0};
                int startIdx = pos * SAMPLES_PER_POSITION;
                int endIdx = startIdx + SAMPLES_PER_POSITION;

                for (int i = startIdx; i < endIdx && i < static_cast<int>(samples.size()); ++i) {
                    sum.x += samples[i].x;
                    sum.y += samples[i].y;
                    sum.z += samples[i].z;
                }

                positionAverages[pos].x = sum.x / SAMPLES_PER_POSITION;
                positionAverages[pos].y = sum.y / SAMPLES_PER_POSITION;
                positionAverages[pos].z = sum.z / SAMPLES_PER_POSITION;

                LOG_INFO("Position " + std::to_string(pos + 1) + " avg: (" 
                    + std::to_string(positionAverages[pos].x) + ", "
                    + std::to_string(positionAverages[pos].y) + ", "
                    + std::to_string(positionAverages[pos].z) + ")");
            }

            /* Compute bias from position averages
             * Expected values at each position (assuming gravity = 9.81 m/s^2):
             * Pos 0 (Z+): ax=0, ay=0, az=+9.81
             * Pos 1 (Z-): ax=0, ay=0, az=-9.81
             * Pos 2 (X+): ax=+9.81, ay=0, az=0
             * Pos 3 (X-): ax=-9.81, ay=0, az=0
             * Pos 4 (Y+): ax=0, ay=+9.81, az=0
             * Pos 5 (Y-): ax=0, ay=-9.81, az=0
             */

            /* Calculate bias (offset from zero) */
            accel_bias[0] = (positionAverages[2].x + positionAverages[3].x) / 2.0f;
            accel_bias[1] = (positionAverages[4].y + positionAverages[5].y) / 2.0f;
            accel_bias[2] = (positionAverages[0].z + positionAverages[1].z) / 2.0f;

            /* Calculate scale factors */
            const float GRAVITY = 9.81f;
            accel_scale[0] = GRAVITY / (std::abs(positionAverages[2].x - accel_bias[0]) + 
                                       std::abs(positionAverages[3].x - accel_bias[0])) * 2.0f;
            accel_scale[1] = GRAVITY / (std::abs(positionAverages[4].y - accel_bias[1]) + 
                                       std::abs(positionAverages[5].y - accel_bias[1])) * 2.0f;
            accel_scale[2] = GRAVITY / (std::abs(positionAverages[0].z - accel_bias[2]) + 
                                       std::abs(positionAverages[1].z - accel_bias[2])) * 2.0f;

            LOG_INFO("Accelerometer offsets computed:");
            LOG_INFO("  Bias (m/s^2): [" + std::to_string(accel_bias[0]) + ", " 
                + std::to_string(accel_bias[1]) + ", " + std::to_string(accel_bias[2]) + "]");
            LOG_INFO("  Scale: [" + std::to_string(accel_scale[0]) + ", " 
                + std::to_string(accel_scale[1]) + ", " + std::to_string(accel_scale[2]) + "]");
        }

        bool AccelCalibrator::verifyCalibration() {
            /* Verify that corrected values at each position approach expected gravity */
            bool validCalibration = true;
            const float GRAVITY = 9.81f;
            const float TOLERANCE = 0.5f;  /* Allow ±0.5 m/s^2 */

            for (int pos = 0; pos < 6; ++pos) {
                int idx = pos * SAMPLES_PER_POSITION;
                if (idx >= static_cast<int>(samples.size())) continue;

                /* Apply calibration to this sample */
                float ax = (samples[idx].x - accel_bias[0]) * accel_scale[0];
                float ay = (samples[idx].y - accel_bias[1]) * accel_scale[1];
                float az = (samples[idx].z - accel_bias[2]) * accel_scale[2];

                /* Calculate magnitude */
                float magnitude = std::sqrt(ax*ax + ay*ay + az*az);

                /* Check if magnitude is close to gravity */
                if (std::abs(magnitude - GRAVITY) > TOLERANCE) {
                    LOG_WARN("Position " + std::to_string(pos + 1) + " magnitude " 
                        + std::to_string(magnitude) + " outside tolerance");
                    validCalibration = false;
                }
            }

            return validCalibration;
        }

        bool AccelCalibrator::validatePosition(const Vector3& reading) {
            /* Determine expected axis and direction for current position */
            int expectedAxis = currentPosition / 2;  /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0;  /* 0,2,4 are positive; 1,3,5 are negative */

            float dominantValue = 0.0f;
            float absX = std::abs(reading.x);
            float absY = std::abs(reading.y);
            float absZ = std::abs(reading.z);

            /* Find which axis has the largest magnitude */
            int actualAxis = 0;
            float maxAbs = absX;
            if (absY > maxAbs) {
                actualAxis = 1;
                maxAbs = absY;
            }
            if (absZ > maxAbs) {
                actualAxis = 2;
                maxAbs = absZ;
            }

            /* Check if the dominant axis matches expected axis */
            if (actualAxis != expectedAxis) {
                return false;  /* Wrong axis dominant */
            }

            /* Get the value on the expected axis */
            if (expectedAxis == 0) {
                dominantValue = reading.z;
            } else if (expectedAxis == 1) {
                dominantValue = reading.x;
            } else {
                dominantValue = reading.y;
            }

            /* Check if sign matches expected direction */
            bool isPositive = dominantValue > 0.0f;
            if (isPositive != expectedPositive) {
                return false;  /* Wrong direction */
            }

            /* Check if magnitude on dominant axis exceeds threshold */
            if (std::abs(dominantValue) < ORIENTATION_THRESHOLD) {
                return false;  /* Not strong enough - insufficient gravity reading */
            }

            return true;  /* Position is valid */
        }

        void AccelCalibrator::provideOrientationGuidance(const Vector3& reading) {
            float absX = std::abs(reading.x);
            float absY = std::abs(reading.y);
            float absZ = std::abs(reading.z);

            /* Find dominant axis */
            int dominantAxis = 0;
            float maxAbs = absX;
            if (absY > maxAbs) {
                dominantAxis = 1;
                maxAbs = absY;
            }
            if (absZ > maxAbs) {
                dominantAxis = 2;
                maxAbs = absZ;
            }

            int expectedAxis = currentPosition / 2;  /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0;

            std::cout << "\n" << std::string(50, '!') << "\n";
            std::cout << "ORIENTATION MISMATCH DETECTED\n";

            /* Provide guidance based on what axis is currently dominant */
            const char* axisNames[] = {"Z", "X", "Y"};
            const char* positionNames[] = {
                "Face Up (Z+)", "Face Down (Z-)",
                "Right Side (X+)", "Left Side (X-)",
                "Front Side (Y+)", "Back Side (Y-)"
            };

            if (dominantAxis != expectedAxis) {
                std::cout << "Wrong axis! Currently detecting " << axisNames[dominantAxis] << "-axis as dominant.\n";
                std::cout << "Expected " << positionNames[currentPosition] << "\n";
                std::cout << "Rotate the device to align with the correct axis.\n";
            } else {
                /* Same axis but wrong direction */
                float dominantValue = 0.0f;
                if (expectedAxis == 0) dominantValue = reading.z;
                else if (expectedAxis == 1) dominantValue = reading.x;
                else dominantValue = reading.y;

                if ((dominantValue > 0) != expectedPositive) {
                    std::cout << "Wrong direction! Flip the device 180 degrees.\n";
                    std::cout << "Expected " << positionNames[currentPosition] << "\n";
                }
            }

            std::cout << std::string(50, '!') << "\n";
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
