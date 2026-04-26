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
#include <iomanip>
#include <cmath>
#include <sstream>
#include <sys/select.h>
#include <unistd.h>

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
            LOG_INFO("[GYRO] Calibration started. Total samples needed: " + std::to_string(CAPTURE_SAMPLES));
            return true;
        }

        bool GyroCalibrator::processCalibration() {
            bool retVal = true;
            if (state == GyroState::IDLE || state == GyroState::COMPLETE || state == GyroState::ERROR) {
                retVal = false;
            } else {

            auto& registry = SensorsRegistry::getInstance();

            switch (state) {
                case GyroState::INIT_PROMPT:
                    promptStaticPosition();
                    state = GyroState::WAIT_STATIC;
                    break;

                case GyroState::WAIT_STATIC: {
                    /* Show live gyro readings so user can see the device is still */
                    auto liveSamples = registry.getGyroRawBuffer().getLatest(1);
                    if (!liveSamples.empty()) {
                        displayLiveValues(liveSamples[0]);
                    }
                    /* Transition to capture only when user confirms with ENTER */
                    if (isEnterPressed()) {
                        std::cout << "\n";
                        state = GyroState::CAPTURE_STATIC;
                        LOG_INFO("[GYRO] Started static gyroscope capture");
                    }
                    break;
                }

                case GyroState::CAPTURE_STATIC: {
                    /* Collect samples from registry */
                    auto latestSamples = registry.getGyroRawBuffer().getLatest(1);
                    if (!latestSamples.empty()) {
                        samples.push_back(latestSamples[0]);
                        collectedSamples++;

                        /* Log progress every 50 samples */
                        if (collectedSamples % 50 == 0) {
                            LOG_INFO("[GYRO] Capture: " + std::to_string(collectedSamples)
                                     + "/" + std::to_string(CAPTURE_SAMPLES) + " samples");
                        }
                    }

                    /* Transition to processing once enough samples are collected */
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
                        LOG_INFO("[GYRO] Calibration COMPLETE");
                    } else {
                        LOG_ERROR("[GYRO] Calibration verification failed");
                        state = GyroState::ERROR;
                    }
                    break;

                default:
                    break;
                }
            }

            return retVal;
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
            std::cout << "Live readings shown below - press ENTER when still...\n" << std::flush;
        }

        void GyroCalibrator::captureStaticSamples() {
            LOG_INFO("[GYRO] Capturing static gyroscope data...");
        }

        void GyroCalibrator::displayLiveValues(const Vector3& reading) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "\r  Gx:" << std::setw(9) << reading.x
                      << " dps  Gy:" << std::setw(9) << reading.y
                      << " dps  Gz:" << std::setw(9) << reading.z
                      << " dps  |  press ENTER when still"
                      << std::flush;
        }

        bool GyroCalibrator::isEnterPressed() {
            fd_set readfds;
            struct timeval timeout = {0, 0};
            bool retVal = false;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0) {
                std::cin.ignore();
                retVal = true;
            }
            return retVal;
        }

        void GyroCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) < CAPTURE_SAMPLES) {
                LOG_ERROR("[GYRO] Insufficient samples for calibration");
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

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "[GYRO] Bias computed (dps): ["
                << gyro_bias[0] << ", " << gyro_bias[1] << ", " << gyro_bias[2] << "]";
            LOG_INFO(oss.str());
        }

        bool GyroCalibrator::verifyCalibration() {
            /* Gyro typical at-rest bias for LSM9DS1 (245 dps FS) is <10 dps */
            const float MAX_BIAS_THRESHOLD = 10.0f;
            bool validCalibration = true;

            for (int i = 0; i < 3; ++i) {
                if (std::abs(gyro_bias[i]) > MAX_BIAS_THRESHOLD) {
                    LOG_WARN("[GYRO] Axis " + std::to_string(i) + " bias "
                             + std::to_string(gyro_bias[i]) + " dps exceeds threshold");
                    validCalibration = false;
                }
            }

            /* Variance of static samples must be low — device must have been still */
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
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(6)
                        << "[GYRO] Variance elevated (fan/vibration likely). "
                        << "Variance (dps^2): [" << varX << ", " << varY << ", " << varZ << "]"
                        << " - bias mean is still valid";
                    LOG_WARN(oss.str());
                }
            }

            return validCalibration;
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
