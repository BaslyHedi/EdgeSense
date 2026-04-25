/**
 * @file CalibDataStore.cpp
 * @author Hedi Basly
 * @brief Implementation of calibration data persistence and file I/O management
 * @date 2026-04-19
 */

#include <EdgeSense/Core/CalibDataStore.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <EdgeSense/Logger/Logger.h>

using namespace EdgeSense::Logger;

namespace EdgeSense {
    namespace Core {

        /* Static member initialization */
        const std::string CalibDataStore::CALIB_DIR = "./CalibData/";
        const std::string CalibDataStore::DEFAULT_BIN_FILE = CalibDataStore::CALIB_DIR + "SensorsCalib.bin";
        const std::string CalibDataStore::DEFAULT_JSON_FILE = CalibDataStore::CALIB_DIR + "SensorsCalib.json";

        CalibDataStore::CalibDataStore() {
            ensureCalibDirExists();
        }

        bool CalibDataStore::ensureCalibDirExists() {
            bool retVal = true;
            try {
                std::filesystem::create_directories(CALIB_DIR);
            } catch (const std::exception& e) {
                LOG_ERROR("Failed to create calibration directory: " + std::string(e.what()));
                retVal = false;
            }
            return retVal;
        }

        uint32_t CalibDataStore::generateSessionId() {
            /* Generate a unique session ID based on current timestamp */
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch()
            ).count();
            
            /* Simple hash: take lower 32 bits of timestamp */
            return static_cast<uint32_t>(timestamp & 0xFFFFFFFFUL);
        }

        std::string CalibDataStore::getEffectiveFilename(const std::string& filename, bool isBinary) const {
            if (!filename.empty()) {
                return CALIB_DIR + filename;
            }
            return isBinary ? DEFAULT_BIN_FILE : DEFAULT_JSON_FILE;
        }

        bool CalibDataStore::save(const FullCalibration& calib, const std::string& filename) {
            std::string filePath = getEffectiveFilename(filename, true);
            bool retVal = false;

            std::ofstream outFile(filePath, std::ios::binary);
            if (!outFile.is_open()) {
                LOG_ERROR("Failed to open calibration file for writing: " + filePath);
            } else {
                /* Write the entire FullCalibration structure as binary */
                outFile.write(reinterpret_cast<const char*>(&calib), sizeof(FullCalibration));

                if (!outFile.good()) {
                    LOG_ERROR("Failed to write calibration data to file: " + filePath);
                } else {
                    LOG_INFO("Calibration data saved to " + filePath + " (Session ID: " + std::to_string(calib.session_id) + ")");
                    /* Also export to JSON for human verification */
                    exportToJson(calib, "");
                    retVal = true;
                }
                outFile.close();
            }

            return retVal;
        }

        bool CalibDataStore::load(FullCalibration& calib, const std::string& filename) {
            std::string filePath = getEffectiveFilename(filename, true);
            bool retVal = false;

            std::ifstream inFile(filePath, std::ios::binary);
            if (!inFile.is_open()) {
                LOG_WARN("Calibration file not found: " + filePath + ". Using defaults.");
            } else {
                /* Read the entire FullCalibration structure */
                inFile.read(reinterpret_cast<char*>(&calib), sizeof(FullCalibration));

                if (!inFile.good()) {
                    LOG_ERROR("Failed to read calibration data from file: " + filePath);
                } else {
                    LOG_INFO("Calibration data loaded from " + filePath + " (Session ID: " + std::to_string(calib.session_id) + ")");
                    retVal = true;
                }
                inFile.close();
            }

            return retVal;
        }

        bool CalibDataStore::exists(const std::string& filename) const {
            std::string filePath = getEffectiveFilename(filename, true);
            return std::filesystem::exists(filePath);
        }

        bool CalibDataStore::exportToJson(const FullCalibration& calib, const std::string& filename) {
            std::string filePath = getEffectiveFilename(filename, false);
            bool retVal = false;

            std::ofstream outFile(filePath);
            if (!outFile.is_open()) {
                LOG_ERROR("Failed to open JSON file for writing: " + filePath);
            } else {
                /* Write human-readable JSON format */
                outFile << "{\n";
                outFile << "  \"session_id\": " << calib.session_id << ",\n";
                outFile << "  \"timestamp\": \"" << std::chrono::system_clock::now().time_since_epoch().count() << "\",\n";

                outFile << "  \"accelerometer\": {\n";
                outFile << "    \"bias\": [" << std::fixed << std::setprecision(6)
                        << calib.accel_bias[0] << ", " << calib.accel_bias[1] << ", " << calib.accel_bias[2] << "],\n";
                outFile << "    \"scale\": ["
                        << calib.accel_scale[0] << ", " << calib.accel_scale[1] << ", " << calib.accel_scale[2] << "]\n";
                outFile << "  },\n";

                outFile << "  \"gyroscope\": {\n";
                outFile << "    \"bias\": ["
                        << calib.gyro_bias[0] << ", " << calib.gyro_bias[1] << ", " << calib.gyro_bias[2] << "]\n";
                outFile << "  },\n";

                outFile << "  \"magnetometer\": {\n";
                outFile << "    \"hard_iron_bias\": ["
                        << calib.mag_bias[0] << ", " << calib.mag_bias[1] << ", " << calib.mag_bias[2] << "],\n";
                outFile << "    \"soft_iron_scale\": ["
                        << calib.mag_scale[0] << ", " << calib.mag_scale[1] << ", " << calib.mag_scale[2] << "]\n";
                outFile << "  },\n";

                outFile << "  \"environmental\": {\n";
                outFile << "    \"baseline_pressure_hpa\": " << calib.baseline_pressure << ",\n";
                outFile << "    \"baseline_temperature_c\": " << calib.baseline_temperature << "\n";
                outFile << "  }\n";
                outFile << "}\n";

                if (!outFile.good()) {
                    LOG_ERROR("Failed to write JSON calibration data: " + filePath);
                } else {
                    LOG_INFO("Calibration data exported to JSON: " + filePath);
                    retVal = true;
                }
                outFile.close();
            }

            return retVal;
        }

        bool CalibDataStore::importFromJson(FullCalibration& calib, const std::string& filename) {
            std::string filePath = getEffectiveFilename(filename, false);
            bool retVal = false;

            std::ifstream inFile(filePath);
            if (!inFile.is_open()) {
                LOG_ERROR("Failed to open JSON file for reading: " + filePath);
            } else {

            /* Simple JSON value extraction using string parsing */
            std::stringstream buffer;
            buffer << inFile.rdbuf();
            std::string content = buffer.str();
            inFile.close();

            /* Helper lambda to extract float values from JSON */
            auto extractFloat = [&content](const std::string& key) -> float {
                size_t pos = content.find(key);
                if (pos == std::string::npos) return 0.0f;
                
                pos = content.find(":", pos);
                if (pos == std::string::npos) return 0.0f;
                
                pos = content.find_first_of("-0123456789.", pos);
                if (pos == std::string::npos) return 0.0f;
                
                size_t end = content.find_first_of(",]}\"", pos);
                if (end == std::string::npos) return 0.0f;
                
                return std::stof(content.substr(pos, end - pos));
            };

            /* Helper lambda to extract array of 3 floats */
            auto extractArray = [&content](const std::string& key, float* arr) -> bool {
                size_t pos = content.find(key);
                if (pos == std::string::npos) return false;
                
                pos = content.find("[", pos);
                if (pos == std::string::npos) return false;
                
                size_t end = content.find("]", pos);
                if (end == std::string::npos) return false;
                
                std::string arrayStr = content.substr(pos + 1, end - pos - 1);
                
                for (int i = 0; i < 3; ++i) {
                    try {
                        size_t commaPos = arrayStr.find(",");
                        if (i < 2 && commaPos == std::string::npos) return false;
                        
                        std::string numStr = (i < 2) ? arrayStr.substr(0, commaPos) : arrayStr;
                        
                        /* Trim whitespace */
                        numStr.erase(0, numStr.find_first_not_of(" \t\n\r"));
                        numStr.erase(numStr.find_last_not_of(" \t\n\r") + 1);
                        
                        arr[i] = std::stof(numStr);
                        
                        if (i < 2) {
                            arrayStr = arrayStr.substr(commaPos + 1);
                        }
                    } catch (...) {
                        return false;
                    }
                }
                
                return true;
            };

            /* Extract all values */
            try {
                /* Session ID */
                size_t pos = content.find("\"session_id\"");
                if (pos != std::string::npos) {
                    pos = content.find(":", pos);
                    if (pos != std::string::npos) {
                        calib.session_id = std::stoul(content.substr(pos + 1, content.find(",", pos) - pos - 1));
                    }
                }

                /* Accelerometer */
                extractArray("\"bias\"", calib.accel_bias);
                extractArray("\"scale\"", calib.accel_scale);

                /* Gyroscope */
                size_t gyroPos = content.find("\"gyroscope\"");
                if (gyroPos != std::string::npos) {
                    size_t biasStart = content.find("\"bias\"", gyroPos);
                    if (biasStart != std::string::npos) {
                        size_t arrayStart = content.find("[", biasStart);
                        size_t arrayEnd = content.find("]", arrayStart);
                        std::string arrayStr = content.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                        
                        sscanf(arrayStr.c_str(), "%f, %f, %f", 
                               &calib.gyro_bias[0], &calib.gyro_bias[1], &calib.gyro_bias[2]);
                    }
                }

                /* Magnetometer */
                extractArray("\"hard_iron_bias\"", calib.mag_bias);
                extractArray("\"soft_iron_scale\"", calib.mag_scale);

                /* Environmental */
                calib.baseline_pressure = extractFloat("\"baseline_pressure_hpa\"");
                calib.baseline_temperature = extractFloat("\"baseline_temperature_c\"");

                retVal = true;
            } catch (const std::exception& e) {
                LOG_ERROR("Error parsing JSON calibration file: " + std::string(e.what()));
            }

            if (retVal) {
                LOG_INFO("Calibration data imported from JSON: " + filePath);
            }
        }

            return retVal;
        }

    } /* namespace Core */
} /* namespace EdgeSense */