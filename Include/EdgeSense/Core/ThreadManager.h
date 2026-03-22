/**
 * @file Threadmanager.h
 * @author Hedi Basly
 * @brief Header for Threadmanager module
 * @date 2026-02-16
 */

#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include <pthread.h>
#include <ctime>
#include <numeric>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/Sensors/LPS25HB_EnvSens.h>
#include <EdgeSense/Sensors/LSM9DS1_ImuSens.h>

namespace EdgeSense {
    namespace Core {

    class ThreadManager {
        public:
            ThreadManager(HAL::I2cMaster& i2c);
            ~ThreadManager();

            void start();
            void stop();

            /* Telemetry for Jitter Analysis */
            long long getHarvesterMaxJitter() const { return harvesterMaxJitterNs.load(); }
            long long getRefinerMaxJitter() const { return refinerMaxJitterNs.load(); }
            long long getNavigatorMaxJitter() const { return navigatorMaxJitterNs.load(); }

        private:
            void harvesterLoop(); /* 1ms - Raw Data Producer */
            void refinerLoop();   /* 5ms - DSP / Filtering */
            void navigatorLoop(); /* 10ms - Orientation / UI Updates */

            HAL::I2cMaster& i2cBus;
            std::atomic<bool> running{false};
            
            std::thread harvesterThread;
            std::thread refinerThread;
            std::thread navigatorThread;
            
            std::atomic<long long> harvesterMaxJitterNs{0};
            std::atomic<long long> refinerMaxJitterNs{0};
            std::atomic<long long> navigatorMaxJitterNs{0};
        };

    } /* namespace Core */
} /* namespace EdgeSense */
