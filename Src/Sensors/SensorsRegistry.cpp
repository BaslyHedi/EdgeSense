/**
 * @file SensorsRegistry.cpp
 * @author Hedi Basly
 * @brief Implementation of SensorsRegistry module
 * @date 2026-03-22
 */

 #include <EdgeSense/Sensors/SensorsRegistry.h>

namespace EdgeSense {
    namespace Sensors {

    /**
     * @brief Update the filtered accelerometer data
     * @param accex The filtered X-axis acceleration
     * @param accey The filtered Y-axis acceleration
     * @param accez The filtered Z-axis acceleration
     */
    void SensorsRegistry::updateFilteredImuAccel(float accex, float accey, float accez) {
        std::lock_guard<std::mutex> lock(dataMutex);
        f_accex = accex; f_accey = accey; f_accez = accez;
    }

    /**
     * @brief Get the filtered accelerometer data
     * @param accex Reference to store the filtered X-axis acceleration
     * @param accey Reference to store the filtered Y-axis acceleration
     * @param accez Reference to store the filtered Z-axis acceleration
     */
    void SensorsRegistry::getFilteredImuAccel(float& accex, float& accey, float& accez) {
        std::lock_guard<std::mutex> lock(dataMutex);
        accex = f_accex; accey = f_accey; accez = f_accez;
    }

    /**
     * @brief Update the filtered gyroscope data
     * @param gyrox The filtered X-axis angular velocity
     * @param gyroy The filtered Y-axis angular velocity
     * @param gyroz The filtered Z-axis angular velocity
     */
    void SensorsRegistry::updateFilteredImuGyro(float gyrox, float gyroy, float gyroz) {
        std::lock_guard<std::mutex> lock(dataMutex);
        f_gyrox = gyrox; f_gyroy = gyroy; f_gyroz = gyroz;
    }

    /**
     * @brief Get the filtered gyroscope data
     * @param gyrox Reference to store the filtered X-axis angular velocity
     * @param gyroy Reference to store the filtered Y-axis angular velocity
     * @param gyroz Reference to store the filtered Z-axis angular velocity
     */
    void SensorsRegistry::getFilteredImuGyro(float& gyrox, float& gyroy, float& gyroz) {
        std::lock_guard<std::mutex> lock(dataMutex);
        gyrox = f_gyrox; gyroy = f_gyroy; gyroz = f_gyroz;
    }

    /**
     * @brief Update the filtered magnetometer data
     * @param magx The filtered X-axis magnetic field strength
     * @param magy The filtered Y-axis magnetic field strength
     * @param magz The filtered Z-axis magnetic field strength
     */
    void SensorsRegistry::updateFilteredImuMag(float magx, float magy, float magz) {
        std::lock_guard<std::mutex> lock(dataMutex);
        f_magx = magx; f_magy = magy; f_magz = magz;
    }

    /**
     * @brief Get the filtered magnetometer data
     * @param magx Reference to store the filtered X-axis magnetic field strength
     * @param magy Reference to store the filtered Y-axis magnetic field strength
     * @param magz Reference to store the filtered Z-axis magnetic field strength
     */
    void SensorsRegistry::getFilteredImuMag(float& magx, float& magy, float& magz) {
        std::lock_guard<std::mutex> lock(dataMutex);
        magx = f_magx; magy = f_magy; magz = f_magz;
    }

    /**
     * @brief Update the filtered environmental data
     * @param press The filtered pressure
     * @param temp The filtered temperature
     */
    void SensorsRegistry::updateFilteredEnv(float press, float temp) {
        std::lock_guard<std::mutex> lock(dataMutex);
        f_pressure = press;
        f_temp = temp;
    }

    /**
     * @brief Get the filtered environmental data
     * @param press Reference to store the filtered pressure
     * @param temp Reference to store the filtered temperature
     */
    void SensorsRegistry::getFilteredEnv(float& press, float& temp) {
        std::lock_guard<std::mutex> lock(dataMutex);
        press = f_pressure;
        temp = f_temp;
    }

    void SensorsRegistry::updateOrientation(float roll_deg, float pitch_deg, float yaw_deg, bool valid) {
        std::lock_guard<std::mutex> lock(dataMutex);
        f_roll_deg         = roll_deg;
        f_pitch_deg        = pitch_deg;
        f_yaw_deg          = yaw_deg;
        f_orientation_valid = valid;
    }

    void SensorsRegistry::getOrientation(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& valid) {
        std::lock_guard<std::mutex> lock(dataMutex);
        roll_deg  = f_roll_deg;
        pitch_deg = f_pitch_deg;
        yaw_deg   = f_yaw_deg;
        valid     = f_orientation_valid;
    }

    } /* namespace Sensors */
} /* namespace EdgeSense */