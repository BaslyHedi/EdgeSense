/**
 * @file main.cpp
 * @author Hedi Basly
 * @brief Entry point for EdgeSense Application
 * @date 2026-02-16
 */
#include <iostream>
#include <EdgeSense/Logger/Logger.h>

int main() 
{
    auto& logger = EdgeSense::Logger::Logger::getInstance();
    
    LOG_INFO("🚀 EdgeSense starting on Hedi-RPi5!");
    LOG_WARN("Log file will be stored at /var/log/EdgeSenseApp.log");
    
    // Keep it alive briefly to see the output
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    logger.stop();
    return 0;
}
