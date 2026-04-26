/**
 * @file main.cpp
 * @author Hedi Basly
 * @brief Entry point for EdgeSense Application
 * @date 2026-04-19
 */
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/Core/ThreadManager.h>
#include <EdgeSense/Core/SensorManager.h>

/* Handle cases where the hash might not be defined (e.g., building outside git) */
#ifndef GIT_COMMIT_HASH
    #define GIT_COMMIT_HASH "Unknown-Dev"
#endif

using namespace EdgeSense::Core;

int main(int argc, char* argv[]) {
    bool calibrationMode = false;
    
    /* Parse command line arguments */
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--calib" || arg == "-c") {
            calibrationMode = true;
            LOG_INFO("Calibration mode enabled via command line flag");
            break;
        }
    }
    
    ThreadManager ThreadManager;
    SensorManager sensorManager(ThreadManager);
    LOG_DEBUG("EdgeSense Version: " << GIT_COMMIT_HASH);
    LOG_DEBUG("Compiled on: " << __DATE__ << " at " << __TIME__);

    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");

    if (!sensorManager.init()) {
        LOG_ERROR("Failed to initialize SensorManager");
        return 1;
    }
    
    if (calibrationMode) {
        LOG_INFO("Running in CALIBRATION mode");
        sensorManager.runCalibration();
    } else {
        LOG_INFO("Running in APPLICATION mode");
        sensorManager.runApplication();
    }
    
    /* Keep main alive and monitor thread mode */
    while(true) { 
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    return 0;
}
