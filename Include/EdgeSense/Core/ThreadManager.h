/**
 * @file Threadmanager.h
 * @author Hedi Basly
 * @brief Header for Threadmanager module
 * @date 2026-02-16
 */

#pragma once
#include <thread>
#include <atomic>
#include <functional>
#include <pthread.h>


namespace EdgeSense {
    namespace Core {

    enum class Tier { Harvester, Refiner, Navigator };

    /* Using slow sampling rates instead of the desired ones (1/5/10) => (5/25/50) */
    #define HARVESTER_CYCLETIME_MS 5
    #define REFINER_CYCLETIME_MS 25
    #define NAVIGATOR_CYCLETIME_MS 50

    class ThreadManager {
        public:
        /* Define the function signatures for our three tiers */
            using TaskFunc = std::function<void()>;
            ThreadManager();
            ~ThreadManager();

            void start();
            void stop();

            /* Setters for the tasks */
            void setHarvesterTask(TaskFunc task) { harvesterTask = task; }
            void setRefinerTask(TaskFunc task) { refinerTask = task; }
            void setNavigatorTask(TaskFunc task) { navigatorTask = task; }

            /* Jitter Telemetry */
            long long getMaxJitter(Tier tier) const {
                switch (tier) {
                    case Tier::Harvester: return harvesterMaxJitterNs.load();
                    case Tier::Refiner:   return refinerMaxJitterNs.load();
                    case Tier::Navigator: return navigatorMaxJitterNs.load();
                    default: return 0;
                }
            }

        private:
            void harvesterWrapper(); /* Handles timing + Jitter + calls injected task */
            void refinerWrapper();   /* Handles 5ms timing + calls injected task */
            void navigatorWrapper(); /* Handles 10ms timing + calls injected task */

            std::atomic<bool> running{false};
            
            std::thread harvesterThread;
            std::thread refinerThread;
            std::thread navigatorThread;

            TaskFunc harvesterTask;
            TaskFunc refinerTask;
            TaskFunc navigatorTask;

            std::atomic<long long> harvesterMaxJitterNs{0};
            std::atomic<long long> refinerMaxJitterNs{0};
            std::atomic<long long> navigatorMaxJitterNs{0};
        };

    } /* namespace Core */
} /* namespace EdgeSense */
