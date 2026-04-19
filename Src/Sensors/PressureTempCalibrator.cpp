/**
 * @file PressureTempCalibrator.cpp
 * @author Hedi Basly
 * @brief Implementation of pressure/temperature baseline calibration
 * @date 2026-04-19
 */

#include <EdgeSense/Sensors/PressureTempCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <iostream>

using namespace EdgeSense::Logger;

namespace EdgeSense {
    namespace Sensors {

        PressureTempCalibrator::PressureTempCalibrator() 
            : baseline_press(0), baseline_temperature(0), calibrationDone(false) {
        }

        bool PressureTempCalibrator::startCalibration() {
            calibrationDone = false;
            collectedSamples = 0;
            targetSampleCount = BASELINE_SAMPLES;

            LOG_INFO("Pressure/Temperature calibration starting. Target samples: " + std::to_string(BASELINE_SAMPLES));

            std::cout << "\n" << std::string(50, '-') << "\n";
            std::cout << "PRESSURE/TEMPERATURE CALIBRATION\n";
            std::cout << "Capturing baseline readings\n";
            std::cout << "Keep the device stationary\n";
            std::cout << "This will take approximately 2 seconds\n";
            std::cout << std::string(50, '-') << "\n";
            std::cout << "Press ENTER to begin...\n";
            std::cin.ignore();

            captureBaseline();
            return true;
        }

        bool PressureTempCalibrator::processCalibration() {
            if (calibrationDone) {
                return false;
            }

            auto& registry = SensorsRegistry::getInstance();

            /* Collect pressure samples */
            auto pressureSamples = registry.getPressure().getLatest(1);
            if (!pressureSamples.empty()) {
                baseline_press += pressureSamples[0];
            }

            /* Collect temperature samples */
            auto tempSamples = registry.getTemperature().getLatest(1);
            if (!tempSamples.empty()) {
                baseline_temperature += tempSamples[0];
            }

            collectedSamples++;

            /* Check if we have enough samples */
            if (collectedSamples >= BASELINE_SAMPLES) {
                /* Calculate averages */
                baseline_press /= BASELINE_SAMPLES;
                baseline_temperature /= BASELINE_SAMPLES;

                LOG_INFO("Pressure/Temperature baseline captured:");
                LOG_INFO("  Baseline Pressure: " + std::to_string(baseline_press) + " hPa");
                LOG_INFO("  Baseline Temperature: " + std::to_string(baseline_temperature) + " C");

                calibrationDone = true;
            }

            return true;
        }

        bool PressureTempCalibrator::isComplete() const {
            return calibrationDone;
        }

        std::string PressureTempCalibrator::getStateString() const {
            if (!calibrationDone) {
                return "CAPTURING_BASELINE";
            }
            return "COMPLETE";
        }

        void PressureTempCalibrator::captureBaseline() {
            LOG_INFO("Starting pressure/temperature baseline capture");
        }

    } /* namespace Sensors */
} /* namespace EdgeSense */
