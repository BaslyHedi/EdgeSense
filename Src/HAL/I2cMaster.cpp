/**
 * @file I2cMaster.cpp
 * @author Hedi Basly
 * @brief Implementation of I2cMaster module
 * @date 2026-02-16
 */
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Logger/Logger.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // read(), write(), close()
#include <sys/ioctl.h>  // ioctl()
#include <linux/i2c-dev.h> // I2C_SLAVE constants

namespace EdgeSense {
namespace HAL {

/**
 * @brief Constructor: Initialize I2cMaster with the specified bus device path
 * Sets up the I2C master driver with the specified bus device path and resets
 * internal state variables for file descriptor and slave address tracking.
 * @param bus Device path to the I2C bus (e.g., "/dev/i2c-1")
 */
I2cMaster::I2cMaster(const std::string& bus) 
    : busDevice(bus), fileDescriptor(-1), currentSlaveAddr(0) {}

/**
 * @brief Destructor: Clean up I2C bus resources
 * Ensures the I2C bus is properly closed and resources are released before
 * the I2cMaster object is destroyed.
 */
I2cMaster::~I2cMaster() {
    closeBus();
}

/**
 * @brief Open the I2C bus device for read/write operations
 * Opens the I2C bus device specified in the constructor with O_RDWR flags to enable
 * both reading and writing. Logs appropriate messages on success or failure.
 * @return true if bus opened successfully, false otherwise
 * @note The file descriptor is stored in the fileDescriptor member variable
 */
bool I2cMaster::openBus() {
    bool success = true;
    fileDescriptor = open(busDevice.c_str(), O_RDWR);
    if (fileDescriptor < 0) {
        LOG_ERROR("Failed to open I2C bus: " + busDevice);
        success = false;
    }
    LOG_INFO("I2C bus opened successfully: " + busDevice);
    return success;
}

/**
 * @brief Close the I2C bus device and release resources
 * Closes the open file descriptor and resets it to -1 to indicate a closed state.
 * This method is safe to call multiple times and handles the case where the bus
 * is not currently open.
 */
void I2cMaster::closeBus() {
    if (fileDescriptor >= 0) {
        close(fileDescriptor);
        fileDescriptor = -1;
    }
}

/**
 * @brief Select the I2C slave device at the specified address
 * Uses the ioctl() system call with I2C_SLAVE flag to select the target slave device.
 * Caches the current slave address to avoid redundant ioctl calls when selecting the same device repeatedly.
 * @param slaveAddr Slave device address (7-bit format, typically 0x00-0x7F)
 * @return true if slave selected successfully, false otherwise
 * @note The ioctl() call may fail if the bus is not open or if the address is invalid
 */
bool I2cMaster::selectSlave(uint8_t slaveAddr) {
    if (currentSlaveAddr != slaveAddr) {
        if (ioctl(fileDescriptor, I2C_SLAVE, slaveAddr) < 0) {
            LOG_ERROR("Failed to select I2C slave address: 0x" + std::to_string(slaveAddr));
            return false;
        }
        currentSlaveAddr = slaveAddr;
    }
    return true;
}

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
bool I2cMaster::writeByte(uint8_t slaveAddr, uint8_t reg, uint8_t value) {
    bool success = false;

    /* Lock the I2C bus for exclusive access */
    std::lock_guard<std::mutex> lock(busMutex);
    
    if (selectSlave(slaveAddr)) {
        uint8_t buffer[2] = {reg, value};
        if (write(fileDescriptor, buffer, 2) == 2) {
            success = true;
        } else {
            LOG_ERROR("I2C write failed to reg 0x" + std::to_string(reg));
        }
    }
    
    return success;
}

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
bool I2cMaster::readByte(uint8_t slaveAddr, uint8_t reg, uint8_t& value) {
    return readBytes(slaveAddr, reg, &value, 1);
}

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
bool I2cMaster::readBytes(uint8_t slaveAddr, uint8_t reg, uint8_t* data, size_t length) {
    bool success = false;
    
    /* Lock the I2C bus for exclusive access */
    std::lock_guard<std::mutex> lock(busMutex);

    if (selectSlave(slaveAddr)) {
        /* Step 1: Write the register address we want to read from */
        if (write(fileDescriptor, &reg, 1) == 1) {
            /* Step 2: Read the requested data bytes */
            if (read(fileDescriptor, data, length) == static_cast<ssize_t>(length)) {
                success = true;
            } else {
                LOG_ERROR("I2C read failed from reg 0x" + std::to_string(reg));
            }
        } else {
            LOG_ERROR("I2C write (reg selection) failed");
        }
    }
    
    return success;
}

} /* namespace HAL */
} /* namespace EdgeSense */
