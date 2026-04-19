/**
 * @file AccelCalibrator.cpp
 * @author Hedi Basly
 * @brief Implementation of accelerometer calibration via 6-point static capture
 * @date 2026-04-19
 */

#include <EdgeSense/Sensors/AccelCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <sys/select.h>
#include <unistd.h>

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

            std::cout << "\n✓ STEP 1 accepted. Accelerometer calibration initialized.\n";
            std::cout << "[ACCEL] Total samples needed: " << targetSampleCount << "\n\n" << std::flush;
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
                        std::cout << std::fixed << std::setprecision(2);
                        std::cout << "\r[ACCEL] Capturing Position " << (currentPosition + 1) << ": ("
                            << reading.x << ", " << reading.y << ", " << reading.z << ")   " << std::flush;

                        /* Validate position for first few samples (50 samples = ~0.5 sec) */
                        int positionSampleCount = collectedSamples % SAMPLES_PER_POSITION;
                        if (positionSampleCount < 50) {
                            if (!validatePosition(reading)) {
                                /* Position invalid - prompt user once per detection event */
                                if (validationAttempts == 0) {
                                    validationAttempts++;
                                    std::cout << "\n[ACCEL] WARNING: Position " << (currentPosition + 1)
                                        << " appears incorrect. Verify device orientation.\n" << std::flush;
                                    provideOrientationGuidance(reading);

                                    if (!promptRetryOrAbort()) {
                                        std::cout << "[ACCEL] Calibration aborted by user.\n" << std::flush;
                                        state = AccelState::ERROR;
                                        return false;
                                    }
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
                                /* Skip this sample */
                                break;
                            } else {
                                /* Valid position - print "Position OK" message first time only */
                                if (!captureStarted) {
                                    std::cout << "\n[ACCEL] ✓ Position " << (currentPosition + 1)
                                        << " OK - Starting capture (" << SAMPLES_PER_POSITION << " samples)...\n" << std::flush;
                                    captureStarted = true;
                                }
                                validationAttempts = 0;
                            }
                        }

                        samples.push_back(reading);
                        collectedSamples++;
                        
                        /* Show progress every 25 samples */
                        int positionProgress = collectedSamples % SAMPLES_PER_POSITION;
                        if (positionProgress > 0 && positionProgress % 25 == 0) {
                            std::cout << "  " << positionProgress << "/" << SAMPLES_PER_POSITION 
                                << " samples\n" << std::flush;
                        }
                    }

                    /* Check if we have enough samples for this position */
                    if (collectedSamples % SAMPLES_PER_POSITION == 0) {
                        std::cout << "[ACCEL] Position " << (currentPosition + 1) << " capture done.\n"<< std::flush;
                        currentPosition++;
                        if (currentPosition < 6) {
                            /* Move to next position */
                            state = static_cast<AccelState>(static_cast<int>(AccelState::WAIT_POSITION_1) + (currentPosition * 2));
                            validationAttempts = 0;  /* Reset for new position */
                            promptPosition();
                        } else {
                            /* All positions captured - process */
                            std::cout << "\n[ACCEL] All 6 positions captured. Processing offsets...\n" << std::flush;
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
                        std::cout << "\n[ACCEL] ✓✓✓ Calibration COMPLETE ✓✓✓\n" << std::flush;
                    } else {
                        std::cout << "\n[ACCEL] ✗ Verification failed - offsets may be incorrect\n" << std::flush;
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
            std::cout << "\n" << std::string(60, '=') << "\n";
            std::cout << "STEP 2: Position " << (currentPosition + 1) << " of 6\n";
            std::cout << std::string(60, '=') << "\n";
            std::cout << "Instructions: " << positionInstructions[currentPosition] << "\n";
            std::cout << std::string(60, '-') << "\n";
            std::cout << "Align the board, watch the live readings below, then press ENTER...\n" << std::flush;
            /* ENTER is detected non-blocking in the WAIT state to allow live display */
        }

        void AccelCalibrator::capturePosition() {
            /* Feedback for capture start is now handled in the CAPTURE_POSITION case */
        }

        void AccelCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) < SAMPLES_PER_POSITION * 6) {
                std::cout << "[ACCEL] ERROR: Insufficient samples (" << samples.size() << "/" 
                    << (SAMPLES_PER_POSITION * 6) << ")\n" << std::flush;
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

                std::cout << "  Pos " << (pos + 1) << " avg: ("
                    << std::setprecision(2) << std::fixed
                    << positionAverages[pos].x << ", "
                    << positionAverages[pos].y << ", "
                    << positionAverages[pos].z << ")\n";
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

            std::cout << "[ACCEL] Offsets computed:\n";
            std::cout << "  Bias (m/s²):  [" << std::setprecision(4) << std::fixed
                << accel_bias[0] << ", " << accel_bias[1] << ", " << accel_bias[2] << "]\n";
            std::cout << "  Scale factors: [" << accel_scale[0] << ", "
                << accel_scale[1] << ", " << accel_scale[2] << "]\n";
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
                    std::cout << "[ACCEL] Position " << (pos + 1) << " magnitude " 
                        << magnitude << " outside tolerance\n" << std::flush;
                    validCalibration = false;
                }
            }

            return validCalibration;
        }

        bool AccelCalibrator::validatePosition(const Vector3& reading) {
            /* Determine expected axis and direction for current position */
            int expectedAxis = currentPosition / 2;  /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0;  /* 0,2,4 are positive; 1,3,5 are negative */

            float expectedValue = 0.0f;
            
            /* Get the value on the expected axis */
            if (expectedAxis == 0) {
                expectedValue = reading.z;
            } else if (expectedAxis == 1) {
                expectedValue = reading.x;
            } else {
                expectedValue = reading.y;
            }

            /* Check if direction matches expected */
            bool isPositive = expectedValue > 0.0f;
            if (isPositive != expectedPositive) {
                return false;  /* Wrong direction for this axis */
            }

            /* Check if magnitude is strong enough to indicate gravity alignment */
            /* At least ORIENTATION_THRESHOLD (in g units) on expected axis means good alignment (1.0g = 9.81 m/s^2) */
            if (std::abs(expectedValue) < ORIENTATION_THRESHOLD) {
                return false;  /* Not enough gravity on expected axis */
            }

            return true;  /* Position is acceptable */
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
            float  progress    = std::min(alignedValue / 1.0f, 1.0f);
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

        void AccelCalibrator::provideOrientationGuidance(const Vector3& reading) {
            int expectedAxis = currentPosition / 2;  /* 0=Z, 1=X, 2=Y */
            bool expectedPositive = (currentPosition % 2) == 0;

            float expectedValue = 0.0f;
            
            if (expectedAxis == 0) {
                expectedValue = reading.z;
            } else if (expectedAxis == 1) {
                expectedValue = reading.x;
            } else {
                expectedValue = reading.y;
            }

            const char* axisNames[] = {"Z", "X", "Y"};
            const char* positionNames[] = {
                "Face Up (Z+)", "Face Down (Z-)",
                "Right Side (X+)", "Left Side (X-)",
                "Front Side (Y+)", "Back Side (Y-)"
            };

            std::cout << "\n" << std::string(50, '!') << "\n";
            std::cout << "ORIENTATION GUIDANCE\n";

            if (std::abs(expectedValue) < ORIENTATION_THRESHOLD) {
                std::cout << axisNames[expectedAxis] << "-axis gravity reading is too weak (" 
                    << std::abs(expectedValue) << " g).\n";
                std::cout << "Rotate device more to align " << axisNames[expectedAxis] 
                    << "-axis with gravity.\n";
            }

            bool isPositive = expectedValue > 0.0f;
            if (isPositive != expectedPositive) {
                std::cout << axisNames[expectedAxis] << "-axis is pointing the WRONG direction.\n";
                std::cout << "Flip the device 180 degrees around the other axes.\n";
            }

            std::cout << "Expected: " << positionNames[currentPosition] << "\n";
            std::cout << std::string(50, '!') << "\n";
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
