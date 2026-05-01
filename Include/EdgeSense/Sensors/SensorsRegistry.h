/**
 * @file SensorRegistry.h
 * @author Hedi Basly
 * @brief Header for SensorRegistry module
 * @date 2026-03-22
 */

#pragma once
#include <EdgeSense/Utils/CircularBuffer.h>
#include <EdgeSense/Sensors/Sensors.h>
#include <atomic>
#include <mutex>

namespace EdgeSense {
    namespace Sensors {

    class SensorsRegistry {
        public:
            /* Access the single instance (Singleton Pattern) */
            static SensorsRegistry& getInstance() {
                static SensorsRegistry instance;
                return instance;
            }

            /* --- Raw Data Buffers (For 1ms -> 5ms Pipeline) --- */
            Utils::CircularBuffer<Vector3, 50>& getAccelRawBuffer()    { return accelRawBuffer; }
            Utils::CircularBuffer<Vector3, 50>& getGyroRawBuffer()      { return gyroRawBuffer; }
            Utils::CircularBuffer<Vector3, 50>& getMagRawBuffer()       { return magnetoRawBuffer; }
            
            Utils::CircularBuffer<float, 50>& getTemperature() { return tempBuffer; }
            Utils::CircularBuffer<float, 50>& getPressure() { return pressBuffer; }

            /* --- Filtered Data (Thread-Safe "Snapshots" for 10ms+ UI/Math) --- */
            /* Accelerometer data */
            void updateFilteredImuAccel(float accex, float accey, float accez);
            void getFilteredImuAccel(float& accex, float& accey, float& accez);

            /* Gyroscope data */
            void updateFilteredImuGyro(float gyrox, float gyroy, float gyroz);
            void getFilteredImuGyro(float& gyrox, float& gyroy, float& gyroz);

            /* Magnetometer data */
            void updateFilteredImuMag(float magx, float magy, float magz);
            void getFilteredImuMag(float& magx, float& magy, float& magz);

            void updateFilteredEnv(float press, float temp);
            void getFilteredEnv(float& press, float& temp);

            /* --- Orientation Data (written by Navigator / AhrsEngine at PROCESS tier) --- */
            void updateOrientation(float roll_deg, float pitch_deg, float yaw_deg, bool valid);
            void getOrientation(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& valid);

        private:
            SensorsRegistry() = default; /* Private constructor for Singleton */

            /* Buffers for orientation high-speed raw data */
            Utils::CircularBuffer<Vector3, 50> accelRawBuffer;
            Utils::CircularBuffer<Vector3, 50> gyroRawBuffer;
            Utils::CircularBuffer<Vector3, 50> magnetoRawBuffer;
            
            /* Buffers for environmental high-speed raw data */
            Utils::CircularBuffer<float, 50> tempBuffer;
            Utils::CircularBuffer<float, 50> pressBuffer;

            /* Mutex-protected "Last Known Good" values for the UI/Fusion layers */
            std::mutex dataMutex;
            float f_accex, f_accey, f_accez, f_gyrox, f_gyroy, f_gyroz, f_magx, f_magy, f_magz;
            float f_pressure, f_temp;

            /* Orientation snapshots (plain floats — same pattern as IMU/env above) */
            float f_roll_deg        = 0.0f;
            float f_pitch_deg       = 0.0f;
            float f_yaw_deg         = 0.0f;
            bool  f_orientation_valid = false;
        };

    } /* namespace Sensors */
} /* namespace EdgeSense */