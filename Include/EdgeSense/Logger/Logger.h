/**
 * @file Logger.h
 * @author Hedi Basly
 * @brief Header for Logger module
 * @date 2026-02-16
 */
#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <sstream>
#include <fstream>

namespace EdgeSense {
    namespace Logger {

    /**
     * @brief Severity levels for log filtering
     */
    enum class LogLevel {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    /**
     * @brief Singleton Logger class
     * Handles log messages in a background thread to prevent I/O blocking.
     */
    class Logger {
    public:
        /* Delete copy constructor and assignment operator (Singleton rule) */
        Logger(const Logger&) = delete;
        Logger& operator=(const Logger&) = delete;

        static Logger& getInstance();

        /**
         * @brief Adds a message to the logging queue
         * @param level Severity of the log
         * @param message Text to log
         */
        void log(LogLevel level, const std::string& message);

        /**
         * @brief Gracefully shuts down the background thread
         */
        void stop();

    private:
        Logger();  /* Private constructor */
        ~Logger(); /* Private destructor */

        void processLogs();           /* Worker thread function */
        void rotateLog();             /* Log rotation based on file size */
        std::string levelToString(LogLevel level);

        /* Threading members */
        std::queue<std::string> logQueue;
        std::mutex queueMutex;
        std::condition_variable condVar;
        std::thread workerThread;
        std::atomic<bool> running;

        /* Log file management */
        std::ofstream logFile;
        const std::string logPath = "/var/log/EdgeSenseApp.log";
        const uintmax_t maxSize = 5 * 1024 * 1024; /* 5MB Limit */
    };

    /* Shortcut macros — write LOG_INFO("Hello") instead of the full singleton call */
    #define LOG_INFO(msg) EdgeSense::Logger::Logger::getInstance().log(EdgeSense::Logger::LogLevel::INFO, msg)
    #define LOG_WARN(msg) EdgeSense::Logger::Logger::getInstance().log(EdgeSense::Logger::LogLevel::WARNING, msg)
    #define LOG_ERROR(msg) EdgeSense::Logger::Logger::getInstance().log(EdgeSense::Logger::LogLevel::ERROR, msg)

    } /* namespace Logger */
} /* namespace EdgeSense */
