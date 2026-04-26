/**
 * @file CalibratorBase.h
 * @author Hedi Basly
 * @brief Abstract base class for sensor calibration state machines
 * @date 2026-04-12
 */

#pragma once

#include <string>

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Abstract base class defining the calibration interface
         * 
         * All sensor-specific calibrators inherit from this class and must implement
         * a state machine pattern for interactive, guided calibration sequences.
         */
        class CalibratorBase {
        public:
            virtual ~CalibratorBase() = default;

            /**
             * @brief Start the calibration sequence for this sensor
             * @return true if calibration started successfully, false otherwise
             */
            virtual bool startCalibration() = 0;

            /**
             * @brief Process the next step of the calibration state machine
             * Called repeatedly by CalibrationEngine to advance through calibration states
             * @return true if progress was made, false if waiting or complete
             */
            virtual bool processCalibration() = 0;

            /**
             * @brief Check if calibration is complete
             * @return true if all calibration steps are finished
             */
            virtual bool isComplete() const = 0;

            /**
             * @brief Check if calibration has failed
             * @return true if the calibrator is in an error/failed state
             */
            virtual bool isError() const { return false; }

            /**
             * @brief Get human-readable string of current calibration state
             * Used for user feedback during interactive calibration
             * @return State string (e.g., "WAIT_POSITION_1", "PROCESSING", "COMPLETE")
             */
            virtual std::string getStateString() const = 0;

            /**
             * @brief Set the target number of samples to collect
             * @param count Number of samples (varies by sensor type)
             */
            void setSampleCount(int count) { targetSampleCount = count; }

            /**
             * @brief Get the number of samples collected so far
             * @return Current sample count
             */
            int getSampleCount() const { return collectedSamples; }

            /**
             * @brief Get target sample count
             * @return Target samples to collect
             */
            int getTargetSampleCount() const { return targetSampleCount; }

        protected:
            int targetSampleCount = 0;
            int collectedSamples = 0;
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */
