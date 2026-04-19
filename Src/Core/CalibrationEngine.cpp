/**
 * @file CalibrationEngine.cpp
 * @author Hedi Basly
 * @brief Implementation of multi-sensor calibration orchestrator
 * @date 2026-04-12
 */

#include <EdgeSense/Core/CalibrationEngine.h>
#include <EdgeSense/Sensors/CalibratorBase.h>
#include <EdgeSense/Sensors/AccelCalibrator.h>
#include <EdgeSense/Sensors/GyroCalibrator.h>
#include <EdgeSense/Sensors/MagCalibrator.h>
#include <EdgeSense/Sensors/PressureTempCalibrator.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <iostream>
#include <iomanip>
#include <ctime>

using namespace EdgeSense::Logger;
using namespace EdgeSense::Sensors;

namespace EdgeSense {
    namespace Core {

        CalibrationEngine::CalibrationEngine() {
            dataStore = std::make_unique<CalibDataStore>();
            initializeCalibrators();
        }

        CalibrationEngine::~CalibrationEngine() {
            /* Cleanup handled by std::unique_ptr destructors */
        }

        void CalibrationEngine::initializeCalibrators() {
            /* Create calibrator instances */
            accelCalib = std::make_unique<AccelCalibrator>();
            gyroCalib = std::make_unique<GyroCalibrator>();
            magCalib = std::make_unique<MagCalibrator>();
            pressureCalib = std::make_unique<PressureTempCalibrator>();

            /* Build calibration sequence */
            calibrationSequence.clear();
            calibrationSequence.push_back({"Accelerometer", accelCalib.get(), false});
            calibrationSequence.push_back({"Gyroscope", gyroCalib.get(), false});
            calibrationSequence.push_back({"Magnetometer", magCalib.get(), false});
            calibrationSequence.push_back({"Pressure/Temperature", pressureCalib.get(), false});
        }

        bool CalibrationEngine::startCalibrationSequence() {
            bool retVal = true;

            /* Reset state */
            currentSensorIndex = 0;
            calibrationComplete = false;
            currentState = CalibState::WAITING_USER;

            /* Try to load existing calibration as baseline */
            if (!loadExistingCalibration()) {
                LOG_WARN("No existing calibration found. Starting fresh calibration sequence.");
                
                /* Initialize calibData with defaults */
                calibData.session_id = 0;
                for (int i = 0; i < 3; ++i) {
                    calibData.accel_bias[i] = 0.0f;
                    calibData.accel_scale[i] = 1.0f;
                    calibData.gyro_bias[i] = 0.0f;
                    calibData.mag_bias[i] = 0.0f;
                    calibData.mag_scale[i] = 1.0f;
                }
                calibData.baseline_pressure = 0.0f;
                calibData.baseline_temperature = 0.0f;
            }

            /* Verify sequence is set up */
            if (calibrationSequence.empty()) {
                LOG_ERROR("Calibration sequence is empty!");
                currentState = CalibState::ERROR;
                retVal = false;
            } else {
                LOG_INFO("Calibration sequence started. " + std::to_string(calibrationSequence.size()) + " sensors to calibrate.");
                promptUserForNextSensor();
            }

            return retVal;
        }

        void CalibrationEngine::stopCalibration() {
            LOG_WARN("Calibration stopped by user.");
            currentState = CalibState::IDLE;
            calibrationComplete = false;
            currentSensorIndex = 0;
        }

        void CalibrationEngine::harvestRawSamples() {
            /* Only harvest if we're actively calibrating */
            if (currentState != CalibState::IN_PROGRESS || currentSensorIndex >= calibrationSequence.size()) {
                return;
            }

            auto& registry = SensorsRegistry::getInstance();
            Sensors::CalibratorBase* activeCal = calibrationSequence[currentSensorIndex].calibrator;

            /* Route raw samples to the active calibrator based on sensor type */
            if (currentSensorIndex == 0) {
                /* Accelerometer calibration - get latest accel samples */
                auto samples = registry.getAccelRawBuffer().getLatest(1);
                if (!samples.empty()) {
                    /* Calibrator will collect these internally */
                }
            } else if (currentSensorIndex == 1) {
                /* Gyroscope calibration - get latest gyro samples */
                auto samples = registry.getGyroRawBuffer().getLatest(1);
                if (!samples.empty()) {
                    /* Calibrator will collect these internally */
                }
            } else if (currentSensorIndex == 2) {
                /* Magnetometer calibration - get latest mag samples */
                auto samples = registry.getMagRawBuffer().getLatest(1);
                if (!samples.empty()) {
                    /* Calibrator will collect these internally */
                }
            } else if (currentSensorIndex == 3) {
                /* Environmental calibration - get latest pressure/temp samples */
                auto pressureSamples = registry.getPressure().getLatest(1);
                auto tempSamples = registry.getTemperature().getLatest(1);
                if (!pressureSamples.empty() && !tempSamples.empty()) {
                    /* Calibrator will collect these internally */
                }
            }
        }

        void CalibrationEngine::processCalibration() {
            /* Guard against invalid state */
            if (currentState == CalibState::IDLE || currentState == CalibState::ERROR || 
                currentState == CalibState::COMPLETE || currentSensorIndex >= calibrationSequence.size()) {
                return;
            }

            Sensors::CalibratorBase* activeCal = calibrationSequence[currentSensorIndex].calibrator;

            /* Process the active calibrator's state machine */
            if (currentState == CalibState::WAITING_USER) {
                /* User confirmed - start the calibration */
                if (activeCal->startCalibration()) {
                    currentState = CalibState::IN_PROGRESS;
                    LOG_INFO("Started calibration for: " + calibrationSequence[currentSensorIndex].name);
                } else {
                    LOG_ERROR("Failed to start calibration for: " + calibrationSequence[currentSensorIndex].name);
                    currentState = CalibState::ERROR;
                }
            } else if (currentState == CalibState::IN_PROGRESS) {
                /* Advance the calibrator's state machine */
                activeCal->processCalibration();

                /* Check if this sensor's calibration is complete */
                if (activeCal->isComplete()) {
                    LOG_INFO("Calibration complete for: " + calibrationSequence[currentSensorIndex].name);
                    calibrationSequence[currentSensorIndex].isComplete = true;

                    /* Move to next sensor */
                    transitionToNextSensor();
                }
            }
        }

        void CalibrationEngine::transitionToNextSensor() {
            currentSensorIndex++;

            if (currentSensorIndex < calibrationSequence.size()) {
                /* More sensors to calibrate */
                currentState = CalibState::WAITING_USER;
                promptUserForNextSensor();
            } else {
                /* All sensors calibrated - save and complete */
                if (saveAllCalibrationData()) {
                    currentState = CalibState::COMPLETE;
                    calibrationComplete = true;
                    LOG_INFO("ALL CALIBRATION COMPLETE. Data saved to disk.");
                } else {
                    currentState = CalibState::ERROR;
                    LOG_ERROR("Failed to save calibration data!");
                }
            }
        }

        void CalibrationEngine::promptUserForNextSensor() {
            if (currentSensorIndex >= calibrationSequence.size()) {
                return;
            }

            std::cout << "\n" << std::string(60, '=') << "\n";
            std::cout << "CALIBRATION SEQUENCE STEP " << (currentSensorIndex + 1) 
                      << " of " << calibrationSequence.size() << "\n";
            std::cout << "Sensor: " << calibrationSequence[currentSensorIndex].name << "\n";
            std::cout << std::string(60, '=') << "\n";
            std::cout << getStatusMessage() << "\n";
            std::cout << "\nPress ENTER to begin calibration for " 
                      << calibrationSequence[currentSensorIndex].name << "...\n";
            std::cin.ignore();
        }

        bool CalibrationEngine::saveAllCalibrationData() {
            bool retVal = true;

            /* Retrieve calibration data from each completed calibrator */
            if (calibrationSequence.size() > 0 && calibrationSequence[0].isComplete) {
                /* Accel */
                auto accelCal = static_cast<AccelCalibrator*>(calibrationSequence[0].calibrator);
                if (accelCal) {
                    accelCal->getCalibrationData(calibData.accel_bias, calibData.accel_scale);
                }
            }

            if (calibrationSequence.size() > 1 && calibrationSequence[1].isComplete) {
                /* Gyro */
                auto gyroCal = static_cast<GyroCalibrator*>(calibrationSequence[1].calibrator);
                if (gyroCal) {
                    gyroCal->getCalibrationData(calibData.gyro_bias);
                }
            }

            if (calibrationSequence.size() > 2 && calibrationSequence[2].isComplete) {
                /* Mag */
                auto magCal = static_cast<MagCalibrator*>(calibrationSequence[2].calibrator);
                if (magCal) {
                    magCal->getCalibrationData(calibData.mag_bias, calibData.mag_scale);
                }
            }

            if (calibrationSequence.size() > 3 && calibrationSequence[3].isComplete) {
                /* Pressure/Temp */
                auto pressCal = static_cast<PressureTempCalibrator*>(calibrationSequence[3].calibrator);
                if (pressCal) {
                    pressCal->getCalibrationData(&calibData.baseline_pressure, &calibData.baseline_temperature);
                }
            }

            /* Generate session ID and save */
            calibData.session_id = static_cast<uint32_t>(std::time(nullptr)) & 0xFFFFFFFFUL;

            if (!dataStore->save(calibData, "")) {
                LOG_ERROR("Failed to save calibration data to binary file!");
                retVal = false;
            }

            return retVal;
        }

        bool CalibrationEngine::loadExistingCalibration() {
            if (!dataStore->exists("")) {
                return false;
            }

            if (!dataStore->load(calibData, "")) {
                return false;
            }

            LOG_INFO("Existing calibration loaded from disk (Session ID: " + std::to_string(calibData.session_id) + ")");
            return true;
        }

        std::string CalibrationEngine::getCurrentSensorCalibrating() const {
            if (currentSensorIndex < calibrationSequence.size()) {
                return calibrationSequence[currentSensorIndex].name;
            }
            return "None";
        }

        std::string CalibrationEngine::getStatusMessage() const {
            std::string status;

            switch (currentState) {
                case CalibState::IDLE:
                    status = "Calibration not started.";
                    break;
                case CalibState::IN_PROGRESS:
                    status = "Calibration in progress for: " + getCurrentSensorCalibrating();
                    if (currentSensorIndex < calibrationSequence.size()) {
                        auto cal = calibrationSequence[currentSensorIndex].calibrator;
                        status += "\nCollected samples: " + std::to_string(cal->getSampleCount()) + 
                                  " / " + std::to_string(cal->getTargetSampleCount());
                    }
                    break;
                case CalibState::WAITING_USER:
                    status = "Ready to calibrate: " + getCurrentSensorCalibrating();
                    break;
                case CalibState::COMPLETE:
                    status = "All calibration sequences completed successfully!";
                    break;
                case CalibState::ERROR:
                    status = "ERROR: Calibration failed!";
                    break;
            }

            return status;
        }

        void CalibrationEngine::applyCalibrationOffsets(Sensors::Vector3& accel,
                                                        Sensors::Vector3& gyro,
                                                        Sensors::Vector3& mag) {
            /* Apply accelerometer calibration */
            for (int i = 0; i < 3; ++i) {
                float* accelPtr = (i == 0) ? &accel.x : (i == 1) ? &accel.y : &accel.z;
                *accelPtr = (*accelPtr - calibData.accel_bias[i]) * calibData.accel_scale[i];
            }

            /* Apply gyroscope calibration (bias only) */
            gyro.x -= calibData.gyro_bias[0];
            gyro.y -= calibData.gyro_bias[1];
            gyro.z -= calibData.gyro_bias[2];

            /* Apply magnetometer calibration (hard-iron + soft-iron) */
            for (int i = 0; i < 3; ++i) {
                float* magPtr = (i == 0) ? &mag.x : (i == 1) ? &mag.y : &mag.z;
                *magPtr = (*magPtr - calibData.mag_bias[i]) * calibData.mag_scale[i];
            }
        }

        void CalibrationEngine::applyEnvironmentalCalibration(float& pressure, float& temperature) {
            /* For environmental sensors, we just log the baseline but don't apply correction */
            /* Pressure and temperature are typically stable enough without active correction */
            /* This method exists for future use if differential corrections are needed */
        }

    } /* namespace Core */
} /* namespace EdgeSense */
