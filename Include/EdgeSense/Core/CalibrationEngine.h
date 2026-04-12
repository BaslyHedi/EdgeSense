/**
 * @file CalibrationEngine.h
 * @author Hedi Basly
 * @brief Orchestrator for multi-sensor calibration sequence
 * @date 2026-04-12
 */

#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <EdgeSense/Sensors/Sensors.h>
#include <EdgeSense/Core/CalibDataStore.h>

/* Forward declarations */
namespace EdgeSense {
    namespace Sensors {
        class CalibratorBase;
        class AccelCalibrator;
        class GyroCalibrator;
        class MagCalibrator;
        class PressureTempCalibrator;
    }
    namespace Core {
        class ThreadManager;
    }
}

namespace EdgeSense {
    namespace Core {

        /**
         * @brief Overall state of the calibration process
         */
        enum class CalibState {
            IDLE,              /* No calibration in progress */
            IN_PROGRESS,       /* Currently calibrating a sensor */
            WAITING_USER,      /* Waiting for user input/confirmation */
            COMPLETE,          /* All calibration finished successfully */
            ERROR              /* Calibration failed */
        };

        /**
         * @brief Main calibration orchestrator (Singleton)
         * 
         * Manages the entire multi-sensor calibration sequence. Coordinates
         * individual sensor calibrators, handles state transitions, and manages
         * data persistence.
         * 
         * Transitions ThreadManager between APP and CALIB execution modes.
         */
        class CalibrationEngine {
        public:
            /**
             * @brief Get singleton instance
             * @return Reference to the singleton CalibrationEngine
             */
            static CalibrationEngine& getInstance() {
                static CalibrationEngine instance;
                return instance;
            }

            /* Prevent copy and assignment */
            CalibrationEngine(const CalibrationEngine&) = delete;
            CalibrationEngine& operator=(const CalibrationEngine&) = delete;

            /**
             * @brief Start the multi-sensor calibration sequence
             * 
             * Initializes all calibrators and begins the sequence in order:
             * Accel → Gyro → Mag → Pressure/Temp
             * 
             * @return true if calibration sequence started successfully
             */
            bool startCalibrationSequence();

            /**
             * @brief Stop ongoing calibration (early exit)
             */
            void stopCalibration();

            /**
             * @brief Check if entire calibration sequence is complete
             * @return true if all sensors have been calibrated
             */
            bool isCalibrationComplete() const { return calibrationComplete; }

            /**
             * @brief Called by ThreadManager harvest task to collect raw sensor samples
             * Routes samples to the active calibrator
             */
            void harvestRawSamples();

            /**
             * @brief Called by ThreadManager process task to advance calibration state machines
             * Processes collected samples and advances to next calibration steps
             */
            void processCalibration();

            /**
             * @brief Get current calibration state
             * @return Current CalibState
             */
            CalibState getState() const { return currentState; }

            /**
             * @brief Get name of sensor currently being calibrated
             * @return Sensor name string (e.g., "Accelerometer", "Idle")
             */
            std::string getCurrentSensorCalibrating() const;

            /**
             * @brief Get detailed status message for user display
             * @return Human-readable status string
             */
            std::string getStatusMessage() const;

            /**
             * @brief Apply loaded calibration offsets to sensor readings
             * Used during normal operation to correct sensor data
             * 
             * @param accel Reference to acceleration vector (in-place correction)
             * @param gyro Reference to gyro vector (in-place correction)
             * @param mag Reference to magnetometer vector (in-place correction)
             */
            void applyCalibrationOffsets(Sensors::Vector3& accel,
                                        Sensors::Vector3& gyro,
                                        Sensors::Vector3& mag);

            /**
             * @brief Apply loaded calibration to environmental readings
             * @param pressure Reference to pressure value (in-place correction)
             * @param temperature Reference to temperature value (in-place correction)
             */
            void applyEnvironmentalCalibration(float& pressure, float& temperature);

            /**
             * @brief Get the loaded calibration data
             * @return Reference to FullCalibration struct
             */
            const FullCalibration& getCalibrationData() const {
                return calibData;
            }

        private:
            CalibrationEngine();  /* Private constructor for Singleton */
            ~CalibrationEngine();

            CalibState currentState = CalibState::IDLE;
            bool calibrationComplete = false;

            /* Calibrator instances */
            std::unique_ptr<Sensors::AccelCalibrator> accelCalib;
            std::unique_ptr<Sensors::GyroCalibrator> gyroCalib;
            std::unique_ptr<Sensors::MagCalibrator> magCalib;
            std::unique_ptr<Sensors::PressureTempCalibrator> pressureCalib;

            /* Data store for persistence */
            std::unique_ptr<CalibDataStore> dataStore;

            /* Calibration sequence management */
            FullCalibration calibData;
            
            struct SensorCalibInfo {
                std::string name;
                Sensors::CalibratorBase* calibrator;
                bool isComplete;
            };

            std::vector<SensorCalibInfo> calibrationSequence;
            size_t currentSensorIndex = 0;

            /**
             * @brief Prompt user for confirmation before moving to next sensor
             */
            void promptUserForNextSensor();

            /**
             * @brief Transition to next sensor in calibration sequence
             */
            void transitionToNextSensor();

            /**
             * @brief Save all calibration data to persistent storage
             * @return true if save successful
             */
            bool saveAllCalibrationData();

            /**
             * @brief Load existing calibration from disk (if available)
             * @return true if load successful, false if no calibration exists yet
             */
            bool loadExistingCalibration();

            /**
             * @brief Initialize all calibrator instances
             */
            void initializeCalibrators();
        };

    } /* namespace Core */
} /* namespace EdgeSense */
