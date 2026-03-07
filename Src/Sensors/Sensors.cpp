/**
 * @file Sensors.cpp
 * @author Hedi Basly
 * @brief Implementation of Sensors abstraction classes 
 * @date 2026-03-07
 */

#include <string>
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Logger/Logger.h>

namespace EdgeSense {
    namespace Sensors {

        /**
         * @brief Top-level Base Class for all sensors
         */
        class Sensor {
        public:
            virtual ~Sensor() = default;

            // Pure Virtual Functions (Abstraction)
            virtual bool initialize() = 0;
            virtual void update() = 0;

            // Getters (Encapsulation)
            std::string getName() const { return name; }
            uint8_t getAddress() const { return address; }

        protected:
            std::string name;
            uint8_t address;
            EdgeSense::HAL::I2cMaster& i2cBus; // Reference to shared hardware bus
        };

        /**
         * @brief Abstract EnvSensor class for temperature, pressure, humidity sensors.
         */
        class EnvSensors : public Sensor {
        public:
            using Sensor::Sensor; // Inherit Constructor

            // Interface for Environmental Data
            virtual float getTemperature() const = 0;
            virtual float getPressure() const { return 0.0f; }
            virtual float getHumidity() const { return 0.0f; }
        };

        struct Vector3 { float x, y, z; };
        /**
         * @brief Abstract ImuSensor class for accelerometer, gyroscope, magnetometer sensors.
         */
        class ImuSensors : public Sensor {
        public:
            using Sensor::Sensor; // Inherit Constructor

            // Interface for Motion Data
            virtual Vector3 getAcceleration() const = 0;
            virtual Vector3 getGyroscope() const = 0;
            virtual Vector3 getMagnetometer() const { return {0.0f, 0.0f, 0.0f}; }
        };
    } // namespace Sensors
} // namespace EdgeSense
