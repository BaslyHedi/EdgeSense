/**
 * @file MagCalibrator.cpp
 * @author Hedi Basly
 * @brief Implementation of magnetometer calibration via 3D rotation capture
 * @date 2026-04-19
 */

#include <EdgeSense/Sensors/MagCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace EdgeSense::Logger;

namespace EdgeSense {
    namespace Sensors {

        MagCalibrator::MagCalibrator() 
            : state(MagState::IDLE) {
        }

        bool MagCalibrator::startCalibration() {
            state = MagState::INIT_PROMPT;
            samples.clear();
            collectedSamples = 0;
            targetSampleCount = CAPTURE_SAMPLES;

            std::cout << "\n✓ STEP 1 accepted. Magnetometer calibration initialized.\n";
            std::cout << "[MAG] Total samples needed: " << CAPTURE_SAMPLES << "\n\n" << std::flush;
            promptRotationSequence();
            return true;
        }

        bool MagCalibrator::processCalibration() {
            if (state == MagState::IDLE || state == MagState::COMPLETE || state == MagState::ERROR) {
                return false;
            }

            auto& registry = SensorsRegistry::getInstance();

            switch (state) {
                case MagState::INIT_PROMPT:
                    state = MagState::WAIT_ROTATION;
                    break;

                case MagState::WAIT_ROTATION:
                    /* User confirmed - start capture */
                    state = MagState::CAPTURE_ROTATION;
                    LOG_INFO("Started magnetometer 3D rotation capture");
                    break;

                case MagState::CAPTURE_ROTATION: {
                    /* Collect samples from registry */
                    auto latestSamples = registry.getMagRawBuffer().getLatest(1);
                    if (!latestSamples.empty()) {
                        samples.push_back(latestSamples[0]);
                        collectedSamples++;
                    }

                    /* Check if we have enough samples */
                    if (collectedSamples >= CAPTURE_SAMPLES) {
                        state = MagState::PROCESSING;
                        computeOffsets();
                    }
                    break;
                }

                case MagState::PROCESSING:
                    state = MagState::COMPUTE_SCALE;
                    fitEllipsoid();
                    break;

                case MagState::COMPUTE_SCALE:
                    state = MagState::VERIFY;
                    if (verifyCalibration()) {
                        state = MagState::COMPLETE;
                        LOG_INFO("Magnetometer calibration COMPLETE");
                    } else {
                        LOG_ERROR("Magnetometer calibration verification failed");
                        state = MagState::ERROR;
                    }
                    break;

                default:
                    break;
            }

            return true;
        }

        bool MagCalibrator::isComplete() const {
            return state == MagState::COMPLETE;
        }

        std::string MagCalibrator::getStateString() const {
            switch (state) {
                case MagState::IDLE:
                    return "IDLE";
                case MagState::INIT_PROMPT:
                    return "INITIALIZING";
                case MagState::WAIT_ROTATION:
                    return "WAITING_FOR_ROTATION";
                case MagState::CAPTURE_ROTATION:
                    return "CAPTURING_3D_ROTATION";
                case MagState::PROCESSING:
                    return "COMPUTING_HARD_IRON";
                case MagState::COMPUTE_SCALE:
                    return "COMPUTING_SOFT_IRON";
                case MagState::VERIFY:
                    return "VERIFYING";
                case MagState::COMPLETE:
                    return "COMPLETE";
                case MagState::ERROR:
                    return "ERROR";
                default:
                    return "UNKNOWN";
            }
        }

        void MagCalibrator::promptRotationSequence() {
            std::cout << "\n" << std::string(50, '-') << "\n";
            std::cout << "MAGNETOMETER CALIBRATION\n";
            std::cout << "Rotate the device slowly in all directions\n";
            std::cout << "Make figure-8 motions to cover all orientations\n";
            std::cout << "This will take approximately 8 seconds\n";
            std::cout << std::string(50, '-') << "\n";
            std::cout << "Press ENTER when ready...\n";
            std::cin.ignore();
        }

        void MagCalibrator::captureRotationSamples() {
            LOG_INFO("Capturing magnetometer rotation data...");
        }

        void MagCalibrator::computeOffsets() {
            if (static_cast<int>(samples.size()) < CAPTURE_SAMPLES) {
                LOG_ERROR("Insufficient samples for magnetometer calibration");
                return;
            }

            /* Hard-iron calibration using min-max method */
            float minX = 999999, maxX = -999999;
            float minY = 999999, maxY = -999999;
            float minZ = 999999, maxZ = -999999;

            for (const auto& sample : samples) {
                minX = std::min(minX, sample.x);
                maxX = std::max(maxX, sample.x);
                minY = std::min(minY, sample.y);
                maxY = std::max(maxY, sample.y);
                minZ = std::min(minZ, sample.z);
                maxZ = std::max(maxZ, sample.z);
            }

            /* Hard-iron offset is the center of the min-max sphere */
            mag_bias[0] = (maxX + minX) / 2.0f;
            mag_bias[1] = (maxY + minY) / 2.0f;
            mag_bias[2] = (maxZ + minZ) / 2.0f;

            LOG_INFO("Magnetometer hard-iron bias computed (min-max method):");
            LOG_INFO("  Hard-iron offset (µT): [" + std::to_string(mag_bias[0]) + ", " 
                + std::to_string(mag_bias[1]) + ", " + std::to_string(mag_bias[2]) + "]");
        }

        void MagCalibrator::fitEllipsoid() {
            /* Simplified soft-iron calibration using axis magnitude scaling */
            if (static_cast<int>(samples.size()) < CAPTURE_SAMPLES) {
                LOG_ERROR("Insufficient samples for ellipsoid fitting");
                return;
            }

            /* Calculate radius for each axis (distance from hard-iron center) */
            float radX = 999999, radY = 999999, radZ = 999999;
            
            for (const auto& sample : samples) {
                float dx = std::abs(sample.x - mag_bias[0]);
                float dy = std::abs(sample.y - mag_bias[1]);
                float dz = std::abs(sample.z - mag_bias[2]);

                radX = std::max(radX, dx);
                radY = std::max(radY, dy);
                radZ = std::max(radZ, dz);
            }

            /* Soft-iron scale: normalize to the largest radius */
            float maxRad = std::max({radX, radY, radZ});

            if (maxRad > 0) {
                mag_scale[0] = maxRad / radX;
                mag_scale[1] = maxRad / radY;
                mag_scale[2] = maxRad / radZ;
            } else {
                /* Fallback to no scaling */
                mag_scale[0] = 1.0f;
                mag_scale[1] = 1.0f;
                mag_scale[2] = 1.0f;
            }

            LOG_INFO("Magnetometer soft-iron scale computed (ellipsoid fitting):");
            LOG_INFO("  Soft-iron scale: [" + std::to_string(mag_scale[0]) + ", " 
                + std::to_string(mag_scale[1]) + ", " + std::to_string(mag_scale[2]) + "]");
        }

        bool MagCalibrator::verifyCalibration() {
            /* Verify that corrected samples form a sphere */
            if (static_cast<int>(samples.size()) < CAPTURE_SAMPLES) {
                return false;
            }

            /* Calculate corrected magnitudes */
            float sumRadius = 0;
            float minRadius = 999999, maxRadius = 0;

            for (const auto& sample : samples) {
                /* Apply hard-iron correction */
                float cx = sample.x - mag_bias[0];
                float cy = sample.y - mag_bias[1];
                float cz = sample.z - mag_bias[2];

                /* Apply soft-iron scaling */
                cx *= mag_scale[0];
                cy *= mag_scale[1];
                cz *= mag_scale[2];

                /* Calculate radius */
                float radius = std::sqrt(cx*cx + cy*cy + cz*cz);
                sumRadius += radius;
                minRadius = std::min(minRadius, radius);
                maxRadius = std::max(maxRadius, radius);
            }

            float avgRadius = sumRadius / static_cast<float>(samples.size());
            float radiusVariance = maxRadius - minRadius;

            LOG_INFO("Magnetometer verification statistics:");
            LOG_INFO("  Average radius: " + std::to_string(avgRadius) + " µT");
            LOG_INFO("  Min radius: " + std::to_string(minRadius) + " µT");
            LOG_INFO("  Max radius: " + std::to_string(maxRadius) + " µT");
            LOG_INFO("  Variance: " + std::to_string(radiusVariance) + " µT");

            /* Verification passes if variance is below threshold */
            return radiusVariance < VARIANCE_THRESHOLD;
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
