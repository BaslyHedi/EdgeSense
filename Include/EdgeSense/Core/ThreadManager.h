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

    enum class Tier { HARVESTER, REFINER, PROCESS };
    enum class ExecutionMode { APP, CALIB, IDLE };

    /* Using slow sampling rates instead of the desired ones (1/5/10) => (5/10/10) */
    #define HARVESTER_CYCLETIME_MS 5
    #define REFINER_CYCLETIME_MS 10
    #define PROCESS_CYCLETIME_MS 10

    /* Debug Mode */
    #define DEBUG_MODE true
    #define LOG_DEBUG(msg) do { if (DEBUG_MODE) std::cout << "[DEBUG] " << msg << std::endl; } while(0)

    class ThreadManager {
        public:
            /* Define the function signatures for our 4 tiers */
            using TaskFunc = std::function<void()>;

            ThreadManager();
            ~ThreadManager();

            void start();
            void stop();

            /* Setters for the tasks */
            bool setHarvesterTask(ExecutionMode exMode, TaskFunc task);
            bool setRefinerTask(ExecutionMode exMode, TaskFunc task);
            bool setProcessTask(ExecutionMode exMode, TaskFunc task);

            /* Setter for execution mode */
            void setExecutionMode(ExecutionMode mode);

            /* Getter for execution mode */
            ExecutionMode getExecutionMode() const { return currentMode; }

            /* Jitter Telemetry */
            long long getMaxJitter(Tier tier) const {
                switch (tier) {
                    case Tier::HARVESTER: return harvesterMaxJitterNs.load();
                    case Tier::REFINER:   return refinerMaxJitterNs.load();
                    case Tier::PROCESS: return processMaxJitterNs.load();  
                    default: return 0;
                }
            }

        private:
            void harvesterWrapper(); /* Handles timing + jitter measurement + calls injected task */
            void refinerWrapper();   /* Handles 25ms timing + calls injected task */
            void processWrapper();   /* Handles 50ms timing + calls injected task */
            void WaitUntilNextCycle(struct timespec& next_time, int cycleTimeMs);
            void updateMaxJitter(struct timespec& next_time, std::atomic<long long>& maxJitterNs);

            ExecutionMode currentMode = ExecutionMode::IDLE;

            /* Dual-Logic Containers */
            struct {
               TaskFunc harvest, refine, process;
            } AppLogic;

            struct {
                TaskFunc harvest, process;
            } CalibLogic;
            
            std::thread harvesterThread;
            std::thread refinerThread;
            std::thread processThread;

            std::atomic<long long> harvesterMaxJitterNs{0};
            std::atomic<long long> refinerMaxJitterNs{0};
            std::atomic<long long> processMaxJitterNs{0};

            bool running = false; /* To signal threads to stop */
        };

    } /* namespace Core */
} /* namespace EdgeSense */
