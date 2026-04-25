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
#include <sstream>
#include <sys/select.h>
#include <unistd.h>

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
            captureStarted = false;

            LOG_INFO("[ACCEL] Calibration started.");
            promptPosition();
            return true;
        }

        bool AccelCalibrator::processCalibration() {
            bool retVal = true;
            if (state == AccelState::IDLE || state == AccelState::COMPLETE || state == AccelState::ERROR) {
                retVal = false;
            } else {
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
                    case AccelState::WAIT_POSITION_6: {
                        /* Show live sensor values each cycle so the user can align the board */
                        auto liveSamples = registry.getAccelRawBuffer().getLatest(1);
                        if (!liveSamples.empty()) {
                            displayLiveValues(liveSamples[0]);
                        }
                        /* Transition to capture only when user confirms with ENTER */
                        if (isEnterPressed()) {
                            std::cout << "\n";
                            state = static_cast<AccelState>(static_cast<int>(state) + 1);
                            captureStarted = false;
                        }
                        break;
                    }

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
                                    /* Position invalid — prompt user once per detection event */
                                    if (validationAttempts == 0) {
                                        validationAttempts++;
                                        LOG_WARN("[ACCEL] Position " + std::to_string(currentPosition + 1)
                                            + " incorrect. Verify orientation.");

                                        if (!promptRetryOrAbort()) {
                                            LOG_WARN("[ACCEL] Calibration aborted by user.");
                                            state = AccelState::ERROR;
                                        } else {
                                            /* Discard any samples collected for this position */
                                            int positionStart = currentPosition * SAMPLES_PER_POSITION;
                                            samples.resize(static_cast<size_t>(positionStart));
                                            collectedSamples = positionStart;
                                            captureStarted = false;
                                            validationAttempts = 0;
                                            /* Return to WAIT state so user can reposition before next capture */
                                            state = static_cast<AccelState>(
                                                static_cast<int>(AccelState::WAIT_POSITION_1) + currentPosition * 2
                                            );
                                            promptPosition();
                                        }
                                    }
                                    /* Skip this sample */
                                } else {
                                    /* Valid position — print "Position OK" message first time only */
                                    if (!captureStarted) {
                                        LOG_INFO("[ACCEL] Position " + std::to_string(currentPosition + 1)
                                            + " OK - Starting capture (" + std::to_string(SAMPLES_PER_POSITION) + " samples)...");
                                        captureStarted = true;
                                    }
                                    validationAttempts = 0;
                                    samples.push_back(reading);
                                    collectedSamples++;
                                }
                            } else {
                                samples.push_back(reading);
                                collectedSamples++;
                            }

                            /* Show progress every 25 samples */
                            int positionProgress = collectedSamples % SAMPLES_PER_POSITION;
                            if (positionProgress > 0 && positionProgress % 25 == 0) {
                                LOG_INFO("[ACCEL] Position " + std::to_string(currentPosition + 1)
                                    + ": " + std::to_string(positionProgress) + "/" + std::to_string(SAMPLES_PER_POSITION) + " samples");
                            }
                        }

                        /* Check if we have enough samples for this position */
                        if (collectedSamples % SAMPLES_PER_POSITION == 0 && collectedSamples > 0) {
                            LOG_INFO("[ACCEL] Position " + std::to_string(currentPosition + 1) + " capture done.");
                            currentPosition++;
                            if (currentPosition < 6) {
                                /* Move to next position */
                                state = static_cast<AccelState>(static_cast<int>(AccelState::WAIT_POSITION_1) + (currentPosition * 2));
                                validationAttempts = 0;
                                promptPosition();
                            } else {
                                /* All positions captured — process */
                                LOG_INFO("[ACCEL] All 6 positions captured. Processing offsets...");
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
                            LOG_INFO("[ACCEL] Calibration COMPLETE");
                        } else {
                            LOG_WARN("[ACCEL] Verification failed - offsets may be incorrect");
                            state = AccelState::ERROR;
                        }
                        break;

                    default:
                        break;
                }
            }

            return retVal;
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
            LOG_INFO("[ACCEL] Position " + std::to_string(currentPosition + 1)
                     + "/6: " + positionInstructions[currentPosition]);
        }

        void AccelCalibrator::capturePosition() {
            /* Feedback for capture start is now handled in the CAPTURE_POSITION case */
        }

        void AccelCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) >= SAMPLES_PER_POSITION * 6) {
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

                    std::ostringstream oss;
                    oss << "  Pos " << (pos + 1) << " avg: ("
                        << std::setprecision(2) << std::fixed
                        << positionAverages[pos].x << ", "
                        << positionAverages[pos].y << ", "
                        << positionAverages[pos].z << ")";
                    LOG_INFO(oss.str());
                }

                /* Calculate bias (offset from zero)
                 * Expected values at each position (gravity = 9.81 m/s^2):
                 * Pos 0 (Z+): ax=0, ay=0, az=+9.81
                 * Pos 1 (Z-): ax=0, ay=0, az=-9.81
                 * Pos 2 (X+): ax=+9.81, ay=0, az=0
                 * Pos 3 (X-): ax=-9.81, ay=0, az=0
                 * Pos 4 (Y+): ax=0, ay=+9.81, az=0
                 * Pos 5 (Y-): ax=0, ay=-9.81, az=0
                 */
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

                std::ostringstream oss;
                oss << std::setprecision(4) << std::fixed
                    << "[ACCEL] Offsets computed:"
                    << "  Bias (m/s2): [" << accel_bias[0] << ", " << accel_bias[1] << ", " << accel_bias[2] << "]"
                    << "  Scale: [" << accel_scale[0] << ", " << accel_scale[1] << ", " << accel_scale[2] << "]";
                LOG_INFO(oss.str());
            } else {
                LOG_ERROR("[ACCEL] Insufficient samples (" + std::to_string(samples.size()) + "/"
                    + std::to_string(SAMPLES_PER_POSITION * 6) + ")");
            }
        }

        bool AccelCalibrator::verifyCalibration() {
            /* Verify that corrected values at each position approach expected gravity */
            bool validCalibration = true;
            const float GRAVITY = 9.81f;
            const float TOLERANCE = 0.5f;  /* Allow +-0.5 m/s^2 */

            for (int pos = 0; pos < 6; ++pos) {
                int idx = pos * SAMPLES_PER_POSITION;
                if (idx < static_cast<int>(samples.size())) {
                    /* Apply calibration to this sample */
                    float ax = (samples[idx].x - accel_bias[0]) * accel_scale[0];
                    float ay = (samples[idx].y - accel_bias[1]) * accel_scale[1];
                    float az = (samples[idx].z - accel_bias[2]) * accel_scale[2];

                    /* Calculate magnitude */
                    float magnitude = std::sqrt(ax*ax + ay*ay + az*az);

                    /* Check if magnitude is close to gravity */
                    if (std::abs(magnitude - GRAVITY) > TOLERANCE) {
                        LOG_WARN("[ACCEL] Position " + std::to_string(pos + 1) + " magnitude "
                            + std::to_string(magnitude) + " outside tolerance");
                        validCalibration = false;
                    }
                }
            }

            return validCalibration;
        }

        bool AccelCalibrator::validatePosition(const Vector3& reading) {
            /* Determine expected axis and direction for current position */
            int expectedAxis = currentPosition / 2;       /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0; /* 0,2,4 are positive; 1,3,5 are negative */

            float expectedValue = 0.0f;

            /* Get the value on the expected axis */
            if (expectedAxis == 0) {
                expectedValue = reading.z;
            } else if (expectedAxis == 1) {
                expectedValue = reading.x;
            } else {
                expectedValue = reading.y;
            }

            /* Check direction and magnitude in a single expression */
            bool retVal = ((expectedValue > 0.0f) == expectedPositive)
                       && (std::abs(expectedValue) >= ORIENTATION_THRESHOLD);
            return retVal;
        }

        void AccelCalibrator::displayLiveValues(const Vector3& reading) {
            static const char* axisNames[] = {"Z", "X", "Y"};

            int  expectedAxis     = currentPosition / 2;   /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0;

            float axisValue = 0.0f;
            if (expectedAxis == 0) {
                axisValue = reading.z;
            } else if (expectedAxis == 1) {
                axisValue = reading.x;
            } else {
                axisValue = reading.y;
            }

            float alignedValue = expectedPositive ? axisValue : -axisValue;
            bool  aligned      = alignedValue >= ORIENTATION_THRESHOLD;

            /* Build a 10-char bar showing alignment progress toward 1.0g */
            float  progress    = std::max(0.0f, std::min(alignedValue, 1.0f));
            int    filled      = static_cast<int>(progress * 10.0f);
            std::string bar(static_cast<size_t>(filled), '#');
            bar += std::string(static_cast<size_t>(10 - filled), '-');

            std::cout << std::fixed << std::setprecision(3);
            std::cout << "\r  X:" << std::setw(7) << reading.x
                      << "g  Y:" << std::setw(7) << reading.y
                      << "g  Z:" << std::setw(7) << reading.z
                      << "g  |  " << (expectedPositive ? "+" : "-") << axisNames[expectedAxis]
                      << " [" << bar << "] " << std::setw(5) << alignedValue << "g"
                      << (aligned ? "  OK - press ENTER" : "  tilt board     ")
                      << std::flush;
        }

        bool AccelCalibrator::isEnterPressed() {
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

        bool AccelCalibrator::promptRetryOrAbort() {
            char choice = 'a';
            bool retVal = false;
            std::cout << "Retry this position? (r = retry, a = abort): " << std::flush;
            std::cin >> choice;
            std::cin.ignore(256, '\n'); /* consume trailing newline so next cin.ignore() blocks correctly */
            retVal = (choice == 'r' || choice == 'R');
            return retVal;
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
