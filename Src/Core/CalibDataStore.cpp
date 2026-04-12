/**
 * @file CalibDataStore.h
 * @author Hedi Basly
 * @brief Calibration data persistence and file I/O management
 * @date 2026-04-12
 */

#include <EdgeSense/Core/CalibDataStore.h>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <EdgeSense/Logger/Logger.h>

namespace EdgeSense {
    namespace Core {

        CalibDataStore::CalibDataStore() {
            /* Ensure the calibration directory exists */
            std::filesystem::create_directories("./CalibData/");
        }

        bool CalibDataStore::save(const FullCalibration& calib, const std::string& filename) {
            std::string filePath = filename.empty() ? "calibration_data.bin" : filename;
            filePath = "./CalibData/" + filePath;

            std::ofstream outFile(filePath, std::ios::binary);
            if (!outFile) {
                LOG_ERROR("Failed to open file for writing calibration data: " + filePath);
                return false;
            }

            outFile.write(reinterpret_cast<const char*>(&calib), sizeof(FullCalibration));
            if (!outFile.good()) {
                LOG_ERROR("Failed to write calibration data to file: " + filePath);
                return false;
            }

            outFile.close();
            LOG_INFO("Calibration data saved successfully to " + filePath);
            return true;
        }

        bool CalibDataStore::load(FullCalibration& calib, const std::string& filename) {
            std::string filePath = filename.empty() ? "calibration_data.bin" : filename;
            filePath = "./CalibData/" + filePath;

            std::ifstream inFile(filePath, std::ios::binary);
            if (!inFile) {
                LOG_ERROR("Failed to open calibration file for reading: " + filePath);
                return false;
            }

            inFile.read(reinterpret_cast<char*>(&calib), sizeof(FullCalibration));
            if (!inFile.good()) {
                LOG_ERROR("Failed to read calibration data from file: " + filePath);
                return false;
            }

            inFile.close();
            LOG_INFO("Calibration data loaded successfully from " + filePath);
            return true;
        }

    } /* namespace Core */
} /* namespace EdgeSense */