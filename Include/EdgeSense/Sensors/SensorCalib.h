/**
 * @file SensorCalib.h
 * @author Hedi Basly
 * @brief Header for SensorCalib module
  * This module provides functionality for calibrating the sensors, including:
  * - A `CalibProfile` struct to hold bias and scale factors for each axis of the accelerometer, gyroscope, and magnetometer.
  * - Methods to save and load calibration profiles to/from a file, allowing persistence across sessions.
  * - Interactive calibration routines that guide the user through the necessary steps to capture calibration data for both the accelerometer and magnetometer.
 * @date 2026-04-05
 */

#pragma once
#include <string>
#include <vector>
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Sensors/Sensors.h>

/* * Calibration parameters for all sensors in one struct for easy file I/O 
    * Each sensor gets bias and scale for X/Y/Z axes (except gyro which usually only needs bias) */
struct FullCalibration {
    unsigned int session_id;        /* Unique hash to link .bin and .json */
    float accel_bias[3], accel_scale[3];
    float gyro_bias[3];             /* Gyros only need bias */
    float mag_bias[3], mag_scale[3];
};

class SensorCalib {
public:
    /* Define the path to your new root folder */
    const std::string CALIB_DIR = "./CalibData/";
    const std::string CALIB_FILE_BIN = CALIB_DIR + "imu_offsets.bin";
    const std::string CALIB_FILE_JSON = CALIB_DIR + "imu_offsets.json";

    bool saveToFile(const FullCalibration& calib);
    bool loadFromFile(FullCalibration& calib);
    bool isCalibFileExists() const;

    /* The interactive guide logic */
    void runInteractiveCalibration(I2cMaster& i2c, ImuSensors& sensor)
};