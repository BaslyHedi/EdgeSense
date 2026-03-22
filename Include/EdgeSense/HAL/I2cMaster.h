/**
 * @file I2cMaster.h
 * @author Hedi Basly
 * @brief Header for I2cMaster module
 * @date 2026-02-16
 */
#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <mutex>

namespace EdgeSense {
    namespace HAL {
        /**
         * @class I2cMaster
         * @brief I2C Master driver for communication with slave devices
         * 
         * This class provides a high-level interface to perform I2C bus operations including
         * opening/closing the bus, selecting slave devices, and reading/writing register data.
         * It wraps the Linux I2C device interface (/dev/i2c-*) and provides convenient methods
         * for typical I2C master operations.
         * 
         * Usage example:
         *   I2cMaster i2c("/dev/i2c-1");
         *   if (i2c.openBus()) {
         *       uint8_t data;
         *       i2c.readByte(0x5C, 0x0F, data);  // Read from device 0x5C, register 0x0F
         *       i2c.closeBus();
         *   }
         */
        class I2cMaster {
        public:
            /**
             * @brief Constructor: Initialize I2cMaster with the specified bus device path
             * Sets up the I2C master driver with the specified bus device path and resets
             * internal state variables for file descriptor and slave address tracking.
             * @param busDevice Device path to the I2C bus (e.g., "/dev/i2c-1")
             */
            I2cMaster(const std::string& busDevice = "/dev/i2c-1");
            
            /**
             * @brief Destructor: Clean up I2C bus resources
             * Ensures the I2C bus is properly closed and resources are released before
             * the I2cMaster object is destroyed.
             */
            ~I2cMaster();

            /**
             * @brief Prevent copying (Bus handles are unique)
             * Copy constructor is deleted to prevent multiple I2cMaster instances
             * from managing the same file descriptor resource.
             */
            I2cMaster(const I2cMaster&) = delete;
            
            /**
             * @brief Prevent assignment (Bus handles are unique)
             * Assignment operator is deleted to ensure each I2cMaster instance
             * has exclusive ownership of its file descriptor.
             */
            I2cMaster& operator=(const I2cMaster&) = delete;

            /**
             * @brief Open the I2C bus device for read/write operations
             * Opens the I2C bus device specified in the constructor with O_RDWR flags to enable
             * both reading and writing. Logs appropriate messages on success or failure.
             * @return true if bus opened successfully, false otherwise
             * @note The file descriptor is stored in the fileDescriptor member variable
             */
            bool openBus();
            
            /**
             * @brief Close the I2C bus device and release resources
             * Closes the open file descriptor and resets it to -1 to indicate a closed state.
             * This method is safe to call multiple times and handles the case where the bus
             * is not currently open.
             */
            void closeBus();

            /**
             * @brief Write a single byte to a register on the I2C slave
             * Performs a write operation to the specified slave device register. The operation:
             * 1. Selects the target slave device at the given address
             * 2. Constructs a buffer with the register address and value
             * 3. Writes the buffer to the I2C bus
             * All operations use a single return path for cleaner control flow.
             * @param slaveAddr Slave device address
             * @param reg Register address to write to
             * @param value Byte value to write
             * @return true if write successful, false otherwise
             * @note This is a write-only operation; the slave device must support it
             */
            bool writeByte(uint8_t slaveAddr, uint8_t reg, uint8_t value);
            
            /**
             * @brief Read a single byte from a register on the I2C slave
             * Wrapper function that calls readBytes() with a length of 1 byte. This provides
             * a convenient API for single-byte read operations without exposing buffer management.
             * @param slaveAddr Slave device address
             * @param reg Register address to read from
             * @param value Reference to store the read byte
             * @return true if read successful, false otherwise
             * @see readBytes()
             */
            bool readByte(uint8_t slaveAddr, uint8_t reg, uint8_t& value);
            
            /**
             * @brief Read multiple bytes from a register on the I2C slave
             * Performs a block read operation using the standard I2C repeated-start sequence:
             * 1. Selects the target slave device
             * 2. Writes the register address to initiate a read from that register
             * 3. Reads the specified number of bytes into the provided buffer
             * Uses a single return path pattern with nested conditionals for clean error handling.
             * @param slaveAddr Slave device address
             * @param reg Register address to read from
             * @param data Pointer to buffer to store read bytes (caller must ensure sufficient buffer size)
             * @param length Number of bytes to read
             * @return true if read successful, false otherwise
             * @note Buffer overflow protection is the responsibility of the caller
             */
            bool readBytes(uint8_t slaveAddr, uint8_t reg, uint8_t* data, size_t length);

        private:
            /* I2C bus device path (e.g., "/dev/i2c-1") */
            std::string busDevice;
            
            /* File descriptor for the open I2C bus (-1 if not open) */
            int fileDescriptor;
            
            /* Currently selected slave device address (cached to avoid redundant ioctl calls) */
            uint8_t currentSlaveAddr;

            /* Mutex for protecting I2C bus access */
            std::mutex busMutex;

            /**
             * @brief Select the I2C slave device at the specified address
             * Uses the ioctl() system call with I2C_SLAVE flag to select the target slave device.
             * Caches the current slave address to avoid redundant ioctl calls when selecting the same device repeatedly.
             * @param slaveAddr Slave device address (7-bit format, typically 0x00-0x7F)
             * @return true if slave selected successfully, false otherwise
             * @note The ioctl() call may fail if the bus is not open or if the address is invalid
             */
            bool selectSlave(uint8_t slaveAddr);
        };

    } /* namespace HAL */
} /* namespace EdgeSense */