/**
 * @file ThreadManager.cpp
 * @author Hedi Basly
 * @brief Implementation of ThreadManager module
 * @date 2026-02-16
 */
#include <EdgeSense/Core/ThreadManager.h>
#include <ctime>

namespace EdgeSense {
    namespace Core {

    ThreadManager::ThreadManager() : running(false), harvesterMaxJitterNs(0), refinerMaxJitterNs(0), navigatorMaxJitterNs(0) {}

    ThreadManager::~ThreadManager() { stop(); }

    void ThreadManager::start() {
        if (running) return;
        running = true;

        harvesterThread = std::thread(&ThreadManager::harvesterWrapper, this);
        refinerThread   = std::thread(&ThreadManager::refinerWrapper, this);
        navigatorThread = std::thread(&ThreadManager::navigatorWrapper, this);

        /* --- Set RT Priorities --- */
        sched_param sch;

        /* Harvester: Real-time Critical */
        sch.sched_priority = 95; 
        pthread_setschedparam(harvesterThread.native_handle(), SCHED_FIFO, &sch);
        
        /* Refiner: Real-time Important */
        sch.sched_priority = 80; 
        pthread_setschedparam(refinerThread.native_handle(), SCHED_FIFO, &sch);
        
        /* Navigator: Standard Time / Background */
        sch.sched_priority = 0; 
        pthread_setschedparam(navigatorThread.native_handle(), SCHED_FIFO, &sch);
    }

    void ThreadManager::harvesterWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            next_time.tv_nsec += HARVESTER_CYCLETIME_MS * 1000000; /* 10ms */
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* Jitter Measurement */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            if (diff > harvesterMaxJitterNs) harvesterMaxJitterNs = diff;
            next_time = actual_time;

            /* Execute Injected Logic */
            if (harvesterTask) {
                harvesterTask();
            }
        }
    }

    void ThreadManager::refinerWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            next_time.tv_nsec += REFINER_CYCLETIME_MS * 1000000;; /* 50ms */
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* Jitter Measurement */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            if (diff > refinerMaxJitterNs) refinerMaxJitterNs = diff;
            next_time = actual_time;

            /* Execute Injected Logic */
            if (refinerTask) {
                refinerTask();
            }
        }
    }

    void ThreadManager::navigatorWrapper() {
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            next_time.tv_nsec += NAVIGATOR_CYCLETIME_MS * 1000000;; /* 100ms */
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* Jitter Measurement */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            if (diff > navigatorMaxJitterNs) navigatorMaxJitterNs = diff;
            next_time = actual_time;

            /* Execute Injected Logic */
            if (navigatorTask) {
                navigatorTask();
            }
        }
    }

    void ThreadManager::stop() {
        running = false;
        if (harvesterThread.joinable()) harvesterThread.join();
        if (refinerThread.joinable()) refinerThread.join();
        if (navigatorThread.joinable()) navigatorThread.join();
    }

    } /* namespace Core */
} /* namespace EdgeSense */