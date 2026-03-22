/**
 * @file ThreadManager.cpp
 * @author Hedi Basly
 * @brief Implementation of ThreadManager module
 * @date 2026-02-16
 */
#include <EdgeSense/Core/ThreadManager.h>

namespace EdgeSense {
    namespace Core {

    ThreadManager::ThreadManager(HAL::I2cMaster& i2c) : i2cBus(i2c) {}

    ThreadManager::~ThreadManager() {
        stop();
    }

    void ThreadManager::start() {
        if (running) return;
        running = true;

        /* Start Harvester (1ms) */
        harvesterThread = std::thread(&ThreadManager::harvesterLoop, this);
        
        /* Start Refiner (5ms) */
        refinerThread = std::thread(&ThreadManager::refinerLoop, this);

        /* Start Navigator (10ms) */
        navigatorThread = std::thread(&ThreadManager::navigatorLoop, this);

        /* Set thread priorities using pthreads */
        /* Set RT Priorities */
        sched_param sch;

        /* Harvester: Real-time Critical */
        sch.sched_priority = 90; 
        pthread_setschedparam(harvesterThread.native_handle(), SCHED_FIFO, &sch);
        
        /* Refiner: Real-time Important */
        sch.sched_priority = 80;
        pthread_setschedparam(refinerThread.native_handle(), SCHED_FIFO, &sch);

        /* Navigator: Standard Time / Background */
        sch.sched_priority = 0; 
        pthread_setschedparam(navigatorThread.native_handle(), SCHED_OTHER, &sch);
    }

    void ThreadManager::harvesterLoop() {
        using namespace EdgeSense::Sensors;
        using namespace EdgeSense::HAL;
        using namespace EdgeSense::Logger;

        auto& registry = SensorRegistry::getInstance();

        /* Initialize the I2C Bus on /dev/i2c-1 (Standard RPi 5 pins) */
        I2cMaster i2c("/dev/i2c-1");

        /* Open the Bus */
        if (!i2c.openBus()) {
            LOG_ERROR("CRITICAL: Failed to open I2C bus. Exiting.");
        }

        /* Create Sensors objects */
        std::unique_ptr <EnvSensor> Pi_LPS25HB = std::make_unique<LPS25HB>(i2c);
        std::unique_ptr <ImuSensor> Pi_LSM9DS1 = std::make_unique<LSM9DS1>(i2c);

        /* Intialize the LPS25HB */
        if (!Pi_LPS25HB->initialize()) {
            LOG_ERROR("Failed to initialize LPS25HB!");
        }
        /* Intialize the LSM9DS1 */
        if (!Pi_LSM9DS1->initialize()) {
            LOG_ERROR("Failed to initialize LSM9DS1!");
        }

        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            // target = last_target + 1ms
            next_time.tv_nsec += 1000000;
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            /* Precise sleep */
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* --- MEASURE JITTER --- */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            
            if (diff > harvesterMaxJitterNs) harvesterMaxJitterNs = diff;

            /* --- HARDWARE ACQUISITION --- */
            Pi_LSM9DS1->update();
            registry.getAccelRawBuffer().push(Pi_LSM9DS1->getAcceleration());
            registry.getGyroRawBuffer().push(Pi_LSM9DS1->getGyroscope());
            registry.getMagRawBuffer().push(Pi_LSM9DS1->getMagnetometer());

            Pi_LPS25HB->update();
            registry.getPressure().push(Pi_LPS25HB->getPressure());
            registry.getTemperature().push(Pi_LPS25HB->getTemperature());
        }
    }

    void ThreadManager::refinerLoop() {
        auto& registry = Sensors::SensorRegistry::getInstance();
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            /* 5ms Interval */
            next_time.tv_nsec += 5000000;
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* --- MEASURE JITTER --- */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            
            if (diff > refinerMaxJitterNs) refinerMaxJitterNs = diff;

            /* --- DSP / FILTERING PHASE --- */
            
            /* 1. Process Accelerometer (Average of last 5 samples) */
            auto accelSamples = registry.getAccelRawBuffer().getLatest(5);
            if (!accelSamples.empty()) {
                float sumX = 0, sumY = 0, sumZ = 0;
                for (const auto& s : accelSamples) {
                    sumX += s.x; sumY += s.y; sumZ += s.z;
                }
                float n = static_cast<float>(accelSamples.size());
                registry.updateFilteredImuAccel(sumX/n, sumY/n, sumZ/n);
            }

            /* 2. Process Gyroscope */
            auto gyroSamples = registry.getGyroRawBuffer().getLatest(5);
            if (!gyroSamples.empty()) {
                float sumX = 0, sumY = 0, sumZ = 0;
                for (const auto& s : gyroSamples) {
                    sumX += s.x; sumY += s.y; sumZ += s.z;
                }
                float n = static_cast<float>(gyroSamples.size());
                registry.updateFilteredImuGyro(sumX/n, sumY/n, sumZ/n);
            }

            /* 3. Process Environmentals (Simple pass-through for now) */
            auto pSamples = registry.getPressure().getLatest(1);
            auto tSamples = registry.getTemperature().getLatest(1);
            if (!pSamples.empty() && !tSamples.empty()) {
                registry.updateFilteredEnv(pSamples[0], tSamples[0]);
            }
        }
    }

    void ThreadManager::navigatorLoop() {
        auto& registry = Sensors::SensorRegistry::getInstance();
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (running) {
            /* 10ms Interval (100Hz) */
            next_time.tv_nsec += 10000000;
            if (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            /* --- MEASURE JITTER --- */
            struct timespec actual_time;
            clock_gettime(CLOCK_MONOTONIC, &actual_time);
            
            long long diff = (actual_time.tv_sec - next_time.tv_sec) * 1000000000LL + 
                            (actual_time.tv_nsec - next_time.tv_nsec);
            
            if (diff > navigatorMaxJitterNs) navigatorMaxJitterNs = diff;

            /* --- CONSUMPTION PHASE --- */
            
            /* This is where we will call our Orientation Engine.
            For now, we just 'touch' the data to ensure the bridge is working.
            */
            float ax, ay, az, gx, gy, gz;
            registry.getFilteredImuAccel(ax, ay, az);
            registry.getFilteredImuGyro(gx, gy, gz);

            /* Future: 
            Orientation.update(ax, ay, az, gx, gy, gz);
            Logger.log(Orientation.getYawPitchRoll());
            */
        }
    }

    void ThreadManager::stop() {
        running = false;
        if (harvesterThread.joinable()) harvesterThread.join();
        if (refinerThread.joinable()) refinerThread.join();
    }

    } /* namespace Core */
} /* namespace EdgeSense */