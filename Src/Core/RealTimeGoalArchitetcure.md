# EdgeSense: Real-Time Multi-Rate Sensor Framework
**Project:** RPi5 High-Precision Telemetry & Jitter Analysis  
**Architecture Version:** 1.0  
**Target Platform:** Raspberry Pi 5 (BCM2712)

---

## 1. Executive Summary
EdgeSense is a high-performance C++ framework designed to decouple hardware data acquisition from complex signal processing. The system utilizes a multi-threaded "Producer-Consumer" pipeline to handle inertial (IMU) and environmental data at different sample rates, providing a platform for benchmarking Linux kernel latency and jitter.

## 2. System Architecture

### 2.1 Threading Strategy (The 1-5-10 Rule)
The application is divided into three primary execution domains to balance hardware constraints with processing requirements:

| Thread | Frequency | Priority | Responsibility |
| :--- | :--- | :--- | :--- |
| **Harvester** | 1ms (1kHz) | `SCHED_FIFO` (99) | Raw I2C reads, Jitter measurement, Circular Buffer push. |
| **Refiner** | 5ms (200Hz) | `SCHED_FIFO` (80) | Digital Signal Processing (DSP), Low-Pass Filtering, Decimation. |
| **Navigator**| 10ms (100Hz) | `SCHED_OTHER` (0) | Sensor Fusion (AHRS), Orientation (Pitch/Roll), Logging/GUI. |



### 2.2 Data Flow & Synchronization
To maintain thread safety without excessive blocking, the system employs:
* **Circular Buffers:** Lock-free or Mutex-protected ring buffers for moving `int16_t` raw data from the Harvester to the Refiner.
* **Atomic Snapshots:** `std::atomic` structures for sharing final filtered floats (Temperature, Pressure) with the UI/Logic layers.
* **Bus Protection:** A singleton `I2cMaster` utilizing a `std::mutex` to prevent collision during concurrent device access.

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
1. **`I2cMaster`**: Shared hardware abstraction.
2. **`Sensor` (Base)**: Abstract interface for all hardware.
3. **`SensorRegistry`**: Singleton acting as the central data bridge and thread owner.
4. **`JitterLogger`**: Instrumenting the 1ms loop using `CLOCK_MONOTONIC` to record timing deltas in nanoseconds.



---

## 5. Success Metrics
* **Latency:** Time elapsed from I2C DRDY to Buffer Push.
* **Jitter ($J$):** The variation in the 1ms period ($T_{actual} - 1ms$).
* **Data Integrity:** Zero dropped samples during the "Refiner" decimation process.