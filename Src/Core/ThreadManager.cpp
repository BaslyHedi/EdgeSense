/**
 * @file ThreadManager.cpp
 * @author Hedi Basly
 * @brief Implementation of ThreadManager module
 * @date 2026-02-16
 */
#include <EdgeSense/Core/ThreadManager.h>
#include <EdgeSense/Logger/Logger.h>
#include <ctime>

namespace EdgeSense {
    namespace Core {

    ThreadManager::ThreadManager() : currentMode(ExecutionMode::IDLE), harvesterMaxJitterNs(0), refinerMaxJitterNs(0), navigatorMaxJitterNs(0) {}

    ThreadManager::~ThreadManager() { stop(); }

    void ThreadManager::start() {
        if (currentMode != ExecutionMode::IDLE) return;

        harvesterThread = std::thread(&ThreadManager::harvesterWrapper, this);
        refinerThread   = std::thread(&ThreadManager::refinerWrapper, this);
        processThread = std::thread(&ThreadManager::processWrapper, this);

        /* --- Set RT Priorities --- */
        sched_param sch;

        /* Harvester: Real-time Critical */
        sch.sched_priority = 95; 
        pthread_setschedparam(harvesterThread.native_handle(), SCHED_FIFO, &sch);
        
        /* Refiner: Real-time Important */
        sch.sched_priority = 80; 
        pthread_setschedparam(refinerThread.native_handle(), SCHED_FIFO, &sch);
        
        /* Process: Standard Time / Background */
        sch.sched_priority = 0; 
        pthread_setschedparam(processThread.native_handle(), SCHED_FIFO, &sch);
    }

    void ThreadManager::harvesterWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (currentMode != ExecutionMode::IDLE) {
            if(currentMode == ExecutionMode::CALIB) {
                WaitUntilNextCycle(next_time, HARVESTER_CYCLETIME_MS);
                updateMaxJitter(next_time, harvesterMaxJitterNs);
                /* Execute Calibration Logic */
                if (CalibLogic.harvest) {
                    CalibLogic.harvest();
                }
            } 
            else if (currentMode == ExecutionMode::APP)
            {
                WaitUntilNextCycle(next_time, HARVESTER_CYCLETIME_MS);
                updateMaxJitter(next_time, harvesterMaxJitterNs);

                /* Execute Calibration Logic */
                if (AppLogic.harvest) {
                    AppLogic.harvest();
                }
            }
        }
    }

    void ThreadManager::refinerWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        while (currentMode != ExecutionMode::IDLE) {
            if(currentMode == ExecutionMode::CALIB) {
                WaitUntilNextCycle(next_time, REFINER_CYCLETIME_MS);
                updateMaxJitter(next_time, refinerMaxJitterNs);
                /* No logic for refiner in CALIB mode we need raw data */
            } else if (currentMode != ExecutionMode::APP)
            {
                WaitUntilNextCycle(next_time, REFINER_CYCLETIME_MS);
                updateMaxJitter(next_time, refinerMaxJitterNs);

                /* Execute Calibration Logic */
                if (AppLogic.refine) {
                    AppLogic.refine();
                }
            }
        }
    }

    void ThreadManager::processWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (currentMode != ExecutionMode::IDLE) {
            if(currentMode == ExecutionMode::CALIB) {
                WaitUntilNextCycle(next_time, PROCESS_CYCLETIME_MS);
                updateMaxJitter(next_time, processMaxJitterNs);
                
                /* Execute Calibration Logic */
                if (CalibLogic.process) {
                    CalibLogic.process();
                }
            } else if (currentMode != ExecutionMode::APP)
            {
                WaitUntilNextCycle(next_time, PROCESS_CYCLETIME_MS);
                updateMaxJitter(next_time, processMaxJitterNs);

                /* Execute Application Logic */
                if (AppLogic.process) {
                    AppLogic.process();
                }
            }
        }
    }

    void ThreadManager::stop() {
        currentMode = ExecutionMode::IDLE;
        if (harvesterThread.joinable()) harvesterThread.join();
        if (refinerThread.joinable()) refinerThread.join();
        if (processThread.joinable()) processThread.join();
    }

    void ThreadManager::WaitUntilNextCycle(struct timespec& next_time, int cycleTimeMs) {
        next_time.tv_nsec += cycleTimeMs * 1000000; /* cycleTimeMs ms */
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_nsec -= 1000000000;
            next_time.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    void ThreadManager::updateMaxJitter(struct timespec& next_time, std::atomic<long long>& maxJitterNs) {
        /* Jitter Measurement */
        struct timespec actual_time;
        clock_gettime(CLOCK_MONOTONIC, &actual_time);
        long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                        (actual_time.tv_nsec - next_time.tv_nsec);
        if (diff > maxJitterNs) maxJitterNs = diff;
        next_time = actual_time;
    }

    /* Setters for the tasks */
    bool setHarvesterTask(ExecutionMode exMode, TaskFunc task)
    {
        bool valid = false;
        if(currentMode == ExecutionMode::IDLE) {
            if (exMode == ExecutionMode::APP) {
                AppLogic.harvest = task;
                valid = true;
            } else if (exMode == ExecutionMode::CALIB) {
                CalibLogic.harvest = task;
            }
        } else {
            LOG_ERROR("Cannot set tasks while ThreadManager is running. Please set tasks before calling start().");
        }
        
        return valid;
    }
    bool setRefinerTask(ExecutionMode exMode, TaskFunc task)
    {
        bool valid = false;
        if(currentMode == ExecutionMode::IDLE) {
            if (exMode == ExecutionMode::APP) {
                AppLogic.refine = task;
                valid = true;
            }
        } else {
            LOG_ERROR("Cannot set tasks while ThreadManager is running. Please set tasks before calling start().");
        }
        return valid;
    }
    bool setProcessTask(ExecutionMode exMode, TaskFunc task)
    {
        bool valid = false;
        if(currentMode == ExecutionMode::IDLE) {
            if (exMode == ExecutionMode::APP) {
                AppLogic.process = task;
                valid = true;
            } else if (exMode == ExecutionMode::CALIB) {
                CalibLogic.process = task;
                valid = true;
            }
        } else {
            LOG_ERROR("Cannot set tasks while ThreadManager is running. Please set tasks before calling start().");
        }
        return valid;
    }
    void setExecutionMode(ExecutionMode mode)
    { 
        switch (mode)
        {
            case ExecutionMode::APP:
                LOG_INFO("Switched to APP Mode");
                break;
            case ExecutionMode::CALIB:
                LOG_INFO("Switched to CALIBRATION Mode");
                break;
            case ExecutionMode::IDLE:
                LOG_INFO("Switched to IDLE Mode");
                break;
            default:
                LOG_ERROR("Invalid Execution Mode when setting mode.");
                break;
        }     
        currentMode = mode;
        
        /* Reset Jitter Metrics when mode changes */
        harvesterMaxJitterNs = 0;
        refinerMaxJitterNs = 0;
        processMaxJitterNs = 0;
    }

    } /* namespace Core */
} /* namespace EdgeSense */