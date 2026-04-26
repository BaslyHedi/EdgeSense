/**
 * @file CalibDataStore.h
 * @author Hedi Basly
 * @brief Calibration data persistence and file I/O management
 * @date 2026-04-12
 */

#pragma once

#include <string>
#include <cstdint>

namespace EdgeSense {
    namespace Core {

        /**
         * @brief Complete calibration data for all sensors
         * 
         * Holds bias and scale factors for all IMU sensors.
         * Can be serialized to binary or JSON format for persistence.
         */
        struct FullCalibration {
            uint32_t session_id;           /* Unique hash linking binary and JSON files */
            
            /* Accelerometer calibration (bias + scale) */
            float accel_bias[3];           /* m/s^2 */
            float accel_scale[3];          /* Scale factors (1.0 = no scaling) */
            
            /* Gyroscope calibration (bias only - no scale needed) */
            float gyro_bias[3];            /* rad/s */
            
            /* Magnetometer calibration (hard-iron + soft-iron) */
            float mag_bias[3];             /* Hard-iron offset (µT) */
            float mag_scale[3];            /* Soft-iron scale factors (1.0 = no scaling) */
            
            /* Environmental baseline (optional for reference) */
            float baseline_pressure;       /* hPa */
            float baseline_temperature;    /* °C */
        };

        /**
         * @brief Manages persistent storage of calibration data
         * 
         * Provides methods to save/load calibration data in binary and JSON formats.
         * Files are stored in ./CalibData/ directory with session IDs for tracking.
         */
        class CalibDataStore {
        public:
            CalibDataStore();
            virtual ~CalibDataStore() = default;

            /**
             * @brief Save calibration data to binary file
             * @param calib Reference to FullCalibration struct to save
             * @param filename Optional custom filename (uses default if empty)
             * @return true if save successful, false otherwise
             */
            bool save(const FullCalibration& calib, const std::string& filename = "");

            /**
             * @brief Load calibration data from binary file
             * @param calib Reference to FullCalibration struct to populate
             * @param filename Optional custom filename (uses default if empty)
             * @return true if load successful, false otherwise
             */
            bool load(FullCalibration& calib, const std::string& filename = "");

            /**
             * @brief Check if calibration file exists
             * @param filename Optional custom filename (uses default if empty)
             * @return true if file exists and is readable, false otherwise
             */
            bool exists(const std::string& filename = "") const;

            /**
             * @brief Export calibration data to human-readable JSON format
             * Useful for verification and debugging
             * @param calib Reference to FullCalibration struct to export
             * @param filename JSON output filename
             * @return true if export successful, false otherwise
             */
            bool exportToJson(const FullCalibration& calib, const std::string& filename);

            /**
             * @brief Import calibration data from JSON file
             * @param calib Reference to FullCalibration struct to populate
             * @param filename JSON input filename
             * @return true if import successful, false otherwise
             */
            bool importFromJson(FullCalibration& calib, const std::string& filename);

            /**
             * @brief Get the default binary calibration file path
             * @return Path to default calibration file
             */
            std::string getDefaultBinaryPath() const { return DEFAULT_BIN_FILE; }

            /**
             * @brief Get the default JSON calibration file path
             * @return Path to default JSON file
             */
            std::string getDefaultJsonPath() const { return DEFAULT_JSON_FILE; }

        private:
            static const std::string CALIB_DIR;
            static const std::string DEFAULT_BIN_FILE;
            static const std::string DEFAULT_JSON_FILE;

            /**
             * @brief Ensure calibration directory exists, create if needed
             * @return true if directory exists or was created successfully
             */
            bool ensureCalibDirExists();

            /**
             * @brief Generate a unique session ID based on timestamp
             * @return 32-bit session ID
             */
            uint32_t generateSessionId();

            /**
             * @brief Get effective filename (use default if empty string provided)
             * @param filename User-provided filename
             * @return Effective filename to use
             */
            std::string getEffectiveFilename(const std::string& filename, bool isBinary) const;
        };

    } /* namespace Core */
} /* namespace EdgeSense */
