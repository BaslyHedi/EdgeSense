# EdgeSense: Real-Time Multi-Rate Sensor Framework
**Project:** RPi5 High-Precision Telemetry & Jitter Analysis  
**Architecture Version:** 2.0  
**Target Platform:** Raspberry Pi 5 (BCM2712)

---

## 1. Executive Summary
EdgeSense is a high-performance C++ framework designed to decouple hardware data acquisition from complex signal processing. The system utilizes a multi-threaded "Producer-Consumer" pipeline to handle inertial (IMU) and environmental data at different sample rates, providing a platform for benchmarking Linux kernel latency and jitter.

## 2. System Architecture

### 2.1 Threading Strategy (The 1-5-10 Rule)
The application is divided into three primary execution domains to balance hardware constraints with processing requirements:

| Thread | Frequency | Priority | Scheduler | Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| **Harvester** | 1ms (1kHz) | 95 | `SCHED_FIFO` | Raw I2C reads, Jitter measurement, Circular Buffer push. |
| **Refiner** | 5ms (200Hz) | 80 | `SCHED_FIFO` | Digital Signal Processing (DSP), Low-Pass Filtering, Decimation. |
| **Navigator**| 10ms (100Hz) | 0 | `SCHED_FIFO` | Sensor Fusion (AHRS), Orientation (Pitch/Roll), Logging/GUI. |

### 2.2 Real-Time Scheduling Mechanism
Each wrapper thread (`harvesterWrapper`, `refinerWrapper`, `navigatorWrapper`) implements **absolute time scheduling** using:
- **`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`** for deterministic period enforcement
- **Drift Prevention:** After each execution, `next_time` is re-anchored to the actual wake time to prevent scheduled time from drifting across iterations
- **Jitter Measurement:** Calculated as `(actual_wake_time - scheduled_time)` in nanoseconds, tracking maximum observed jitter (`maxJitterNs`)

This approach ensures periodic execution with minimal accumulated error, critical for the 1ms harvester deadline.

### 2.3 Data Flow & Synchronization
To maintain thread safety without excessive blocking, the system employs:
* **Circular Buffers:** Mutex-protected ring buffers for moving `Vector3` (IMU) and `float` (environmental) raw data from the Harvester to the Refiner.
* **Thread-Safe Snapshots:** Atomic structures for sharing final filtered floats (acceleration, gyroscope, magnetometer, temperature, pressure) with consumer threads.
* **Bus Protection:** A singleton `I2cMaster` utilizing a `std::mutex` to prevent collision during concurrent I2C device access.

---

## 3. Real-Time Performance Roadmap

The project aims to quantify the impact of kernel tuning on the 1ms "Harvester" thread.

### Scenario A: Standard Raspberry Pi OS
* **Kernel:** Mainline Linux.
* **Goal:** Establish a baseline. 
* **Expected Result:** Significant jitter spikes (>2ms) during high CPU or Disk I/O load.

### Scenario B: PREEMPT_RT Kernel
* **Kernel:** Patched with `CONFIG_PREEMPT_RT`.
* **Goal:** Evaluate the impact of a fully preemptible kernel.
* **Expected Result:** Deterministic behavior with worst-case jitter remaining below 100μs.

### Scenario C: Custom Yocto Image
* **Build System:** Yocto Project (Poky/Meta-RaspberryPi).
* **Optimization:** Minimal rootfs, CPU Isolation (`isolcpus`), and Thread Affinity.
* **Goal:** Achieve "Hard Real-Time" performance.

---

## 4. Class Design (POO)

### Core Components
1. **`ThreadManager`**: Owns and manages three periodic threads (Harvester, Refiner, Navigator) with real-time scheduling and built-in jitter tracking.
2. **`I2cMaster`**: Shared hardware abstraction layer with mutex-protected bus access.
3. **`Sensor` (Base)**: Abstract interface for hardware sensors (IMU, Environmental).
4. **`SensorRegistry`**: Singleton providing thread-safe data bridge with circular buffers and atomic snapshots for inter-thread communication.

---

## 5. Success Metrics
* **Latency:** Time elapsed from I2C read to Circular Buffer push.
* **Jitter ($J$):** The variation in the periodic sleep interval (maximum observed deviation from scheduled period).
* **Data Integrity:** Zero dropped samples and monotonic timestamp ordering across all tiers.