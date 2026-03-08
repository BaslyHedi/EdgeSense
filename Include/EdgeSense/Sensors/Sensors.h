/**
 * @file Sensors.h
 * @author Hedi Basly
 * @brief Header for Sensors abstraction classes   
 * @date 2026-02-16
 */

#pragma once

#include <string>
#include <vector>
#include "EdgeSense/HAL/I2cMaster.h"

namespace EdgeSense {
    namespace Sensors {
        /**
         * @brief Top-level Base Class for all sensors (abstract)
          * This class defines the common interface and properties for all sensor types. 
          * It includes:
          * - A constructor that initializes the sensor with a name, I2C address, and reference to the I2C bus.
          * - Pure virtual methods `initialize()` and `update()` that must be implemented by all derived sensor classes.
          * - Getters for sensor name and address to allow access to these properties without exposing internal state.
          * - A protected reference to the I2C bus that derived classes can use for communication with their respective hardware.
         */
        class Sensor {
        public:
            Sensor(const std::string& name, uint8_t address, HAL::I2cMaster& bus)
                : name(name), address(address), i2cBus(bus) {}

            // Interface methods
            virtual bool initialize() = 0; // Pure virtual: Child MUST implement
            virtual void update() = 0;     // Pure virtual: Child MUST implement
            
            std::string getName() const { return name; }
            uint8_t getAddress() const { return address; }

        protected:
            std::string name;
            uint8_t address;
            HAL::I2cMaster& i2cBus; // Children use this to talk to hardware
        };

        /**
         * @brief Middle-layer for Environmental Data
         */
        class EnvSensor : public Sensor {
        public:
            using Sensor::Sensor; 
            
            virtual float getTemperature() const = 0;
            virtual float getPressure() const { return 0.0f; }   // Default if not supported
            virtual float getHumidity() const { return 0.0f; }   // Default if not supported
        };

        /**
         * @brief Middle-layer for Motion Data
         */
        // Simple structure for 3D data (IMU)
        struct Vector3 {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
        };

        class ImuSensor : public Sensor {
        public:
            using Sensor::Sensor;

            virtual Vector3 getAcceleration() const { return {0,0,0}; }
            virtual Vector3 getGyroscope() const { return {0,0,0}; }
            virtual Vector3 getMagnetometer() const { return {0,0,0}; }
        };

    } // namespace Sensors
} // namespace EdgeSense
