/**
 * @file GyroCalibrator.cpp
 * @author Hedi Basly
 * @brief Implementation of gyroscope calibration via static at-rest capture
 * @date 2026-04-19
 */

#include <EdgeSense/Sensors/GyroCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <iostream>
#include <cmath>

using namespace EdgeSense::Logger;

namespace EdgeSense {
    namespace Sensors {

        GyroCalibrator::GyroCalibrator() 
            : state(GyroState::IDLE) {
        }

        bool GyroCalibrator::startCalibration() {
            state = GyroState::INIT_PROMPT;
            samples.clear();
            collectedSamples = 0;
            targetSampleCount = CAPTURE_SAMPLES;

            std::cout << "\n✓ STEP 1 accepted. Gyroscope calibration initialized.\n";
            std::cout << "[GYRO] Total samples needed: " << CAPTURE_SAMPLES << "\n\n" << std::flush;
            promptStaticPosition();
            return true;
        }

        bool GyroCalibrator::processCalibration() {
            if (state == GyroState::IDLE || state == GyroState::COMPLETE || state == GyroState::ERROR) {
                return false;
            }

            auto& registry = SensorsRegistry::getInstance();

            switch (state) {
                case GyroState::INIT_PROMPT:
                    state = GyroState::WAIT_STATIC;
                    break;

                case GyroState::WAIT_STATIC:
                    /* User confirmed - start capture */
                    state = GyroState::CAPTURE_STATIC;
                    LOG_INFO("Started static gyroscope capture");
                    break;

                case GyroState::CAPTURE_STATIC: {
                    /* Collect samples from registry */
                    auto latestSamples = registry.getGyroRawBuffer().getLatest(1);
                    if (!latestSamples.empty()) {
                        samples.push_back(latestSamples[0]);
                        collectedSamples++;
                    }

                    /* Check if we have enough samples */
                    if (collectedSamples >= CAPTURE_SAMPLES) {
                        state = GyroState::PROCESSING;
                        computeOffsets();
                    }
                    break;
                }

                case GyroState::PROCESSING:
                    state = GyroState::VERIFY;
                    if (verifyCalibration()) {
                        state = GyroState::COMPLETE;
                        LOG_INFO("Gyroscope calibration COMPLETE");
                    } else {
                        LOG_ERROR("Gyroscope calibration verification failed");
                        state = GyroState::ERROR;
                    }
                    break;

                default:
                    break;
            }

            return true;
        }

        bool GyroCalibrator::isComplete() const {
            return state == GyroState::COMPLETE;
        }

        std::string GyroCalibrator::getStateString() const {
            switch (state) {
                case GyroState::IDLE:
                    return "IDLE";
                case GyroState::INIT_PROMPT:
                    return "INITIALIZING";
                case GyroState::WAIT_STATIC:
                    return "WAITING_FOR_STATIC_POSITION";
                case GyroState::CAPTURE_STATIC:
                    return "CAPTURING_STATIC_DATA";
                case GyroState::PROCESSING:
                    return "PROCESSING_BIAS";
                case GyroState::VERIFY:
                    return "VERIFYING";
                case GyroState::COMPLETE:
                    return "COMPLETE";
                case GyroState::ERROR:
                    return "ERROR";
                default:
                    return "UNKNOWN";
            }
        }

        void GyroCalibrator::promptStaticPosition() {
            std::cout << "\n" << std::string(50, '-') << "\n";
            std::cout << "GYROSCOPE CALIBRATION\n";
            std::cout << "Place the device on a LEVEL SURFACE\n";
            std::cout << "Keep it completely STILL during capture\n";
            std::cout << "This will take approximately 5 seconds\n";
            std::cout << std::string(50, '-') << "\n";
            std::cout << "Press ENTER when ready...\n";
            std::cin.ignore();
        }

        void GyroCalibrator::captureStaticSamples() {
            LOG_INFO("Capturing static gyroscope data...");
        }

        void GyroCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) < CAPTURE_SAMPLES) {
                LOG_ERROR("Insufficient samples for gyroscope calibration");
                return;
            }

            /* Calculate mean for each axis */
            float sumX = 0, sumY = 0, sumZ = 0;

            for (const auto& sample : samples) {
                sumX += sample.x;
                sumY += sample.y;
                sumZ += sample.z;
            }

            /* Bias is the average of the static measurements */
            gyro_bias[0] = sumX / static_cast<float>(samples.size());
            gyro_bias[1] = sumY / static_cast<float>(samples.size());
            gyro_bias[2] = sumZ / static_cast<float>(samples.size());

            LOG_INFO("Gyroscope bias computed:");
            LOG_INFO("  Bias (dps): [" + std::to_string(gyro_bias[0]) + ", "
                + std::to_string(gyro_bias[1]) + ", " + std::to_string(gyro_bias[2]) + "]");
        }

        bool GyroCalibrator::verifyCalibration() {
            /* Verify that computed bias is reasonable (not too large) */
            /* Gyro driver outputs in dps (0.00875 dps/LSB). LSM9DS1 typical at-rest bias is <5 dps */
            const float MAX_BIAS_THRESHOLD = 10.0f;  /* dps */
            bool validCalibration = true;

            for (int i = 0; i < 3; ++i) {
                if (std::abs(gyro_bias[i]) > MAX_BIAS_THRESHOLD) {
                    std::cout << "[GYRO] Axis " << i << " bias " << gyro_bias[i] 
                        << " exceeds threshold\n" << std::flush;
                    validCalibration = false;
                }
            }

            /* Also check variance of samples - should be very low */
            if (validCalibration && samples.size() > 1) {
                float varX = 0, varY = 0, varZ = 0;

                for (const auto& sample : samples) {
                    varX += (sample.x - gyro_bias[0]) * (sample.x - gyro_bias[0]);
                    varY += (sample.y - gyro_bias[1]) * (sample.y - gyro_bias[1]);
                    varZ += (sample.z - gyro_bias[2]) * (sample.z - gyro_bias[2]);
                }

                varX /= static_cast<float>(samples.size());
                varY /= static_cast<float>(samples.size());
                varZ /= static_cast<float>(samples.size());

                if (varX > VARIANCE_THRESHOLD || varY > VARIANCE_THRESHOLD || varZ > VARIANCE_THRESHOLD) {
                    std::cout << "[GYRO] Variance too high - device may not have been still\n";
                    std::cout << "  Variance (dps^2): [" << varX << ", "
                        << varY << ", " << varZ << "]\n" << std::flush;
                    validCalibration = false;
                }
            }

            return validCalibration;
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
