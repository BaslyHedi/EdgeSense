/**
 * @file Logger.cpp
 * @author Hedi Basly
 * @brief Implementation of Logger module
 * @date 2026-02-16
 */
#include <EdgeSense/Logger/Logger.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <filesystem> // C++17 filesystem library for log rotation
namespace fs = std::filesystem;

namespace EdgeSense {
namespace Logger {

/**
 * @brief Returns the singleton Logger instance
 * Uses Meyer's Singleton pattern with static local variable (thread-safe in C++11+)
 * The instance is created only on first call and persists for the program lifetime.
 * @return Reference to the global Logger instance
 */
Logger& Logger::getInstance() 
{
    static Logger instance;
    return instance;
}

/**
 * @brief Logger constructor
 * Initializes the logging system and spawns a background worker thread.
 * The worker thread will continuously process log messages from the queue
 * without blocking the main application threads, ensuring non-blocking I/O.
 */
Logger::Logger() : running(true) 
{
    // Open file in Append mode
    logFile.open(logPath, std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "\033[31mCRITICAL: Can't open log file at \n" << logPath << "\033[0m" << std::endl;
    }
    // Start the background worker thread to process log messages
    workerThread = std::thread(&Logger::processLogs, this);
}

/**
 * @brief Logger destructor
 * Ensures graceful shutdown of the background worker thread before the object is destroyed.
 * This prevents dangling thread references and ensures all pending log messages are processed.
 */
Logger::~Logger() 
{
    stop();
    if (logFile.is_open()) logFile.close();
}

/**
 * @brief Thread-safe method to enqueue a log message
 * This method:
 * 1. Acquires a lock on the queue to prevent race conditions
 * 2. Captures the current system time with second-level precision
 * 3. Formats the message with timestamp, severity level, and content
 * 4. Enqueues the formatted message for the background thread
 * 5. Notifies the condition variable to wake the worker thread if waiting
 * @param level The severity level (DEBUG, INFO, WARNING, ERROR)
 * @param message The log message text
 * @note Lock guard automatically releases the lock when going out of scope (RAII pattern)
 */
void Logger::log(LogLevel level, const std::string& message) 
{
    std::lock_guard<std::mutex> lock(queueMutex);
    // Get current system time as time_t
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    // Format log message with timestamp and severity level
    std::stringstream finalMessage;
    finalMessage << "[" << std::put_time(std::localtime(&now), "%Y-%m-%d -> %H:%M") << "] " << levelToString(level) << ": " << message;
    // Add formatted message to the queue
    logQueue.push(finalMessage.str());
    // Signal the worker thread that a new message is available
    condVar.notify_one();
}

/**
 * @brief Converts a LogLevel enum to a formatted string with color codes
 * Uses ANSI escape sequences for terminal colors to improve log readability:
 * - DEBUG: No color (default terminal color)
 * - INFO: Green (\033[32m...Green text...\033[0m)
 * - WARNING: Yellow (\033[33m...Yellow text...\033[0m)
 * - ERROR: Red (\033[31m...Red text...\033[0m)
 * The \033[0m sequence resets formatting back to default after the colored text.
 * @param level The LogLevel to convert
 * @return Formatted string with color codes and severity label
 */
std::string Logger::levelToString(LogLevel level) 
{
    switch (level) 
    {
        case LogLevel::DEBUG:   return "[DEBUG]";                               // No color formatting
        case LogLevel::INFO:    return "\033[32m[INFO ]\033[0m";               // Green color
        case LogLevel::WARNING: return "\033[33m[WARN ]\033[0m";               // Yellow color
        case LogLevel::ERROR:   return "\033[31m[ERROR]\033[0m";               // Red color
        default:                return "[LOG  ]";                              // Fallback for unknown levels
    }
}

/**
 * @brief Gracefully stops the logging worker thread
 * This method:
 * 1. Sets the running flag to false, signaling the worker to exit its loop
 * 2. Notifies all waiting threads (wakes up the worker thread)
 * 3. Joins the worker thread (waits for it to finish processing and exit)
 * @note The worker thread will process all remaining queued messages before exiting
 * @note This method should be called before program shutdown to ensure clean exit
 */
void Logger::stop() 
{
    running = false;                    // Signal the worker thread to stop
    condVar.notify_all();               // Wake up the worker thread if it's waiting
    if (workerThread.joinable()) workerThread.join();  // Wait for thread to finish gracefully
}

/**
 * @brief Checks the log file size and rotates it if it exceeds the defined limit
 * This method:
 * 1. Checks if the log file exists and retrieves its size
 * 2. If the file size exceeds the maxSize threshold (e.g., 5MB), it performs log rotation:
 *    a. Closes the current log file handle to release it
 *    b. Renames the existing log file to a backup name (e.g., EdgeSenseApp.log.1)
 *    c. Re-opens a new log file with the original name for fresh logging
 *    d. Optionally logs a system message indicating that rotation occurred
 * @note This method should be called periodically (e.g., after each log entry) to ensure timely rotation
 * @note Log rotation helps prevent unbounded growth of log files and manages disk space effectively
 */
void Logger::rotateLog() {
    if (!fs::exists(logPath)) return;

    // Check if file exceeds 5MB
    if (fs::file_size(logPath) > maxSize) {
        logFile.close(); // Close current handle

        std::string backupPath = logPath + ".1";
        
        // Remove old backup if it exists, then rename current to backup
        if (fs::exists(backupPath)) fs::remove(backupPath);
        fs::rename(logPath, backupPath);

        // Re-open fresh log file
        logFile.open(logPath, std::ios::app);
        
        // Internal log to indicate rotation happened
        logFile << "[SYSTEM] Log rotated due to size limit." << std::endl;
    }
}

/**
 * @brief Worker thread function that continuously processes and outputs log messages
 * This method runs in a separate thread and:
 * 1. Continuously loops while running = true OR there are unprocessed messages
 * 2. Uses unique_lock for flexible lock management with condition variables
 * 3. Waits on the condition variable until:
 *    - A new message arrives (queue is not empty), OR
 *    - The stop() method is called (running becomes false)
 * 4. Processes all queued messages in FIFO order
 * 5. Outputs each message to stdout with newline
 * 
 * The dual condition in the outer while loop ensures:
 * - Even after stop() is called, remaining messages in the queue are processed
 * - No log messages are lost during graceful shutdown
 * @note This runs in the background thread, not the main application thread
 * @note The unique_lock is released during condVar.wait() to allow other threads to enqueue
 */
void Logger::processLogs() 
{
    while (running || !logQueue.empty()) 
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        // Wait until either a message arrives or stop() is called
        condVar.wait(lock, [this] { return !logQueue.empty() || !running; });
        // Process all messages currently in the queue
        while (!logQueue.empty()) 
        {
            std::string message = logQueue.front();
            // 1: Output to console with newline
            std::cout << message << std::endl;
            // 2: Write to log file with newline
            if(logFile.is_open())
            {
                rotateLog(); // Check size before writing
                logFile << message << std::endl;
                logFile.flush(); // Ensure the message is written to disk immediately 
            }
            else
            {
                std::cerr << "\033[31mCRITICAL: Log file is not open. Failed to write log message.\033[0m" << std::endl;
            }
            // Remove the message from the queue after processing
            logQueue.pop();
        }
    }
}

} // namespace Logger
} // namespace EdgeSense
