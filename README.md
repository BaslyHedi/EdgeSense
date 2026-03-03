# EdgeSense

**EdgeSense** is a production-oriented embedded Linux daemon written in modern C++.

The system acquires data from multiple I²C sensors (IMU and environmental sensors),
processes them at deterministic periodic intervals, and exposes runtime control
via an IPC interface.

The project focuses on:

- Deterministic scheduling under Linux
- Clean modular architecture
- Hardware abstraction
- Real-time awareness
- Robust service lifecycle handling



## 🎯 Project Goals
* **Deterministic Performance:** Achieving low-jitter sensor polling using POSIX threads and `SCHED_FIFO` scheduling.
* **Modular Architecture:** Implementing a strict separation of concerns between hardware abstraction (HAL), business logic, and communication layers.
* **Real-Time Analysis:** Comparing execution latency and scheduling jitter between standard Linux kernels and `PREEMPT_RT` patched kernels.
* **Production Readiness:** Utilizing industry-standard tools including CMake cross-compilation, Systemd service integration, and automated deployment.

---

## 🏗 System Architecture

EdgeSense utilizes a layered architecture to ensure scalability and testability:

1.  **Hardware Abstraction Layer (HAL):** C++ wrappers for Linux character devices (I2C/SPI), decoupling physical hardware from logic.
2.  **Sensor Layer:** Specific driver logic for the Raspberry Pi Sensor Hat v2 (IMU, Pressure, Humidity).
3.  **Core Engine:** A high-priority scheduler that manages task execution frequencies and thread synchronization.
4.  **IPC & Telemetry:** A socket-based server enabling external processes to consume real-time sensor data.
5.  **Logging:** A thread-safe, asynchronous logging system designed to minimize interference with real-time tasks.

---

## ⚙️ Key Technical Features

- Deterministic periodic execution using CLOCK_MONOTONIC + TIMER_ABSTIME
- Optional SCHED_FIFO real-time scheduling
- Jitter measurement and statistics
- Signal handling (SIGTERM, SIGHUP)
- Unix Domain Socket IPC
- YAML-based configuration
- Structured logging (spdlog)
- systemd service integration

---

## 📂 Folder Structure

The project follows the "Pitchfork" layout, maintaining symmetry between public interfaces and private implementations.

```text
EdgeSense/
├── CMake/              # Cross-compilation toolchain definitions
├── Config/             # Runtime configuration (YAML/JSON)
├── Include/edgesense/  # Public API headers (Interfaces)
│   ├── Core/           # Scheduler and Task management
│   ├── Hal/            # I2C/GPIO Abstraction interfaces
│   ├── IPC/            # Socket communication interfaces
│   ├── Logger/         # Asynchronous logging definitions
│   ├── Sensors/        # Sensor driver interfaces
│   └── Utils/          # Mathematical and conversion helpers
├── Services/           # Systemd unit files for deployment
├── Src/                # Implementation files (.cpp)
│   ├── Core/           # Real-time execution logic
│   ├── Hal/            # Linux-specific driver implementations
│   ├── IPC/            # Socket server and protocol logic
│   ├── Logger/         # Logging engine implementation
│   ├── Sensors/        # Data acquisition for specific hardware
│   └── Utils/          # Utility and helper logic
├── .gitignore          # Git exclusion rules
├── LICENSE             # Project license (MIT)
└── README.md           # Project documentation
```
---

## 🛠 Development & Build Workflow
### Prerequisites
Host: Ubuntu 22.04+ (or equivalent Linux distribution)

Target: Raspberry Pi 5 (running Raspberry Pi OS or custom Yocto build)

Toolchain: aarch64-linux-gnu-gcc/g++

### Cross-Compilation
EdgeSense is designed to be built on a high-performance host and deployed to a Raspberry Pi 5.