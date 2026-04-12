/**
 * @file main.cpp
 * @author Hedi Basly
 * @brief Entry point for EdgeSense Application
 * @date 2026-02-16
 */
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/Core/ThreadManager.h>
#include <EdgeSense/Core/SensorManager.h>

/* Handle cases where the hash might not be defined (e.g., building outside git) */
#ifndef GIT_COMMIT_HASH
    #define GIT_COMMIT_HASH "Unknown-Dev"
#endif

using namespace EdgeSense::Core;

int main() {
    ThreadManager ThreadManager;
    SensorManager sensorManager(ThreadManager);
    LOG_DEBUG("EdgeSense Version: " << GIT_COMMIT_HASH);
    LOG_DEBUG("Compiled on: " << __DATE__ << " at " << __TIME__);

    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");

    sensorManager.init();
    sensorManager.runApplication();
    
    /* Keep main alive and monitor thread mode */
    while(true) { 
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    return 0;
}
