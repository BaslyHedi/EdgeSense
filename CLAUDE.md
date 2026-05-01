# EdgeSense Project Context

## Project Overview
**EdgeSense** is a high-performance, real-time sensor fusion C++ framework designed for the Raspberry Pi 5. It manages high-frequency 9-DOF data from the LSM9DS1 sensor suite, specifically mitigating OS-level jitter via a strict multi-threaded architecture and real-time scheduling priorities.

## Core Architecture
The framework relies on a decoupled three-tier Producer-Consumer pipeline:
1. **Harvester (1ms / 1kHz):** `SCHED_FIFO` real-time thread. Rapidly reads raw I2C registers to ensure minimal hardware latency.
2. **Refiner (5ms / 200Hz):** Medium-priority thread. Averages and filters raw circular buffers into clean data snapshots.
3. **Navigator (10ms / 100Hz):** `SCHED_OTHER` background thread. Consumes clean data for telemetry, logging, and future AHRS (Orientation) math.

## Sensor Details

### Hardware Sensors
- **LSM9DS1_AccGyro (Accelerometer/Gyroscope):** I2C Address `0x6A` (ID: 0x68). Polled at 1kHz (HARVESTER tier).
  - Accel: ±2g range, 119Hz ODR, 0.000061 m/s²/LSB sensitivity
  - Gyro: 245°/s range, 119Hz ODR, 0.00875°/s/LSB sensitivity
  
- **LSM9DS1_Mag (Magnetometer):** I2C Address `0x1E` (ID: 0x3D). Polled every 10ms to reduce I2C bus load.
  - Mag: ±4 Gauss range, 80Hz ODR, 0.00014 mG/LSB sensitivity
  - Hard-iron calibration offsets applied at read time
  - Soft-iron (ellipsoid) scaling available for advanced use
  
- **LPS25HB (Pressure/Temperature):** I2C Address `0x5C` (ID: 0xBD). Environmental baseline.
  - Pressure: 260-1260 hPa, 25Hz ODR
  - Temperature: -40 to +85°C

### Sensor Hierarchy (Class Abstraction)
```
Sensor (abstract base)
├── ImuSensors (motion data contract)
│   ├── LSM9DS1_AccGyro  → Returns Vector3 accel, Vector3 gyro
│   └── LSM9DS1_Mag      → Returns Vector3 magnetometer
└── EnvSensors (environmental contract)
    └── LPS25HB          → Returns float pressure, float temperature
```

### Sensor Registry Pipeline
All sensors → `SensorsRegistry` (Thread-safe Singleton):
- **Raw Buffers:** CircularBuffer<Vector3/float, 50> for 1ms→5ms Producer-Consumer
  - `accelRawBuffer`, `gyroRawBuffer`, `magnetoRawBuffer` (Vector3)
  - `tempBuffer`, `pressBuffer` (float)
- **Filtered Snapshots:** Mutex-protected floats updated by Refiner at 200Hz
  - Accessed by Navigator without blocking Harvester

## Namespace Organization

```
EdgeSense/
├── Core/
│   ├── ThreadManager.h/cpp       - Real-time thread scheduler
│   ├── SensorManager.h/cpp       - Lifecycle manager for all sensors
│   ├── CalibrationEngine.h/cpp   - Calibration orchestrator (NEW)
│   └── CalibDataStore.h/cpp      - Calibration file I/O (NEW)
├── HAL/
│   └── I2cMaster.h/cpp           - Linux I2C abstraction
├── Sensors/
│   ├── Sensors.h                 - Base class hierarchy (Sensor, ImuSensors, EnvSensors)
│   ├── LSM9DS1_ImuSensAccGyro.h/cpp
│   ├── LSM9DS1_ImuSensMag.h/cpp
│   ├── LPS25HB_EnvSens.h/cpp
│   ├── SensorCalib.h/cpp         - FullCalibration struct + legacy methods
│   ├── SensorsRegistry.h/cpp     - Thread-safe data nexus
│   ├── CalibratorBase.h          - Abstract calibration interface (NEW)
│   ├── AccelCalibrator.h/cpp     - Accelerometer calibration (NEW)
│   ├── GyroCalibrator.h/cpp      - Gyroscope calibration (NEW)
│   ├── MagCalibrator.h/cpp       - Magnetometer calibration (NEW)
│   └── PressureTempCalibrator.h/cpp - Environmental calibration (NEW)
├── Logger/
│   └── Logger.h/cpp              - Unified logging
├── IPC/
│   └── SocketServer.h/cpp        - Inter-process telemetry broadcast
└── Utils/
    ├── CircularBuffer.h          - Generic ring buffer template
    └── Utils.h/cpp               - Helper utilities
```

## Key Classes & Design Patterns

### Core Execution Tier
- `ThreadManager`: Orchestrates three real-time threads using `clock_nanosleep` (TIMER_ABSTIME). Executes injected logic via `std::function<>` lambdas.
  - Supports dual execution modes: `APP` mode (normal operation) and `CALIB` mode (calibration)
  - Tracks jitter per-thread using `std::atomic<long long>`
  - Cycle times: Harvester=5ms (1kHz), Refiner=25ms (200Hz), Process=50ms (100Hz) - slowed for debugging

### Sensor Data Management
- `SensorRegistry`: Thread-safe Singleton providing centralized data hub.
  - **Raw Data Path:** Harvester → CircularBuffer (50-depth) → Refiner → Filtered Snapshots
  - **Synchronization:** Lock-free CircularBuffer for raw data; mutex-protected floats for snapshots
  - **Access Pattern:** Registry isolated from sensor details; SensorManager handles sensor lifecycle
  
- `CircularBuffer<T, N>`: Generic fixed-size ring buffer for zero-reallocation high-frequency data.
  - Provides `push()`, `getLatest(k)`, `size()` operations
  - Used for 1kHz→200Hz producer-consumer decoupling

### Hardware Abstraction
- `I2cMaster`: Linux `/dev/i2c-*` wrapper with `readByte()`, `readBytes()`, `writeByte()` primitives
  - RAII design: opened once at init, shared by reference throughout lifetime
  - Supports configurable baudrate (400kHz recommended for multi-device scenarios)

- `Sensor` (abstract): Defines `initialize()` and `update()` contract for all sensor types
  - Subclasses: `ImuSensors`, `EnvSensors` (intermediate abstractions)

### Calibration Architecture (NEW)

#### Namespaces
- `EdgeSense::Core` - CalibrationEngine, CalibDataStore
- `EdgeSense::Sensors` - CalibratorBase, AccelCalibrator, GyroCalibrator, MagCalibrator, PressureTempCalibrator

#### Components
- `CalibrationEngine`: Singleton orchestrator for multi-sensor calibration sequence.
  - Manages calibration order: Accel → Gyro → Mag → Pressure/Temp
  - Transitions ThreadManager between `ExecutionMode::CALIB` and `ExecutionMode::APP`
  - Coordinates harvest/process tasks via SensorManager callbacks
  - Auto-transitions to APP mode on completion
  
- `CalibratorBase`: Abstract base class defining state machine interface.
  - Pure virtual: `startCalibration()`, `processCalibration()`, `isComplete()`, `getStateString()`
  
- **Sensor-Specific Calibrators** (all in `EdgeSense::Sensors`):
  - `AccelCalibrator`: 6-position static capture (face up/down, left/right, front/back). Computes bias and scale.
  - `GyroCalibrator`: Static at-rest capture (level surface). Computes bias only.
  - `MagCalibrator`: 3D rotation capture (figure-8 patterns). Ellipsoid fitting → hard-iron and soft-iron correction.
  - `PressureTempCalibrator`: Simplified baseline averaging (no state machine).
  
- `CalibDataStore`: Persistent calibration data manager.
  - Methods: `save()`, `load()`, `exists()`, `exportToJson()` for verification
  - File format: Binary (imu_offsets.bin) + JSON text copy (imu_offsets.json)
  - Storage location: `./CalibData/` directory
  
- `FullCalibration` struct: Holds all calibration data for all sensors.
  - session_id (unique hash linking files)
  - accel_bias[3], accel_scale[3]
  - gyro_bias[3]
  - mag_bias[3], mag_scale[3]

## System Rules & User Preferences (CRITICAL)
1. **Comment Style:** ALWAYS use `/* */` for all comments in generated C++ code. Do not use `//`.
2. **Resource Management:** Use RAII for hardware handles, file descriptors, and loggers. Avoid opening files or initiating heavy I/O inside high-frequency real-time loops.
3. **Timing:** Use absolute time computations to prevent clock drift over long runs.
4. **Functions:** All functions have a single entry and exit points. Do not use return in the middle of functions
5. **Senior Engineer Mindset:** Before every design decision, ask: *"What would a senior software engineer do?"* — prefer proven algorithms over custom ones, separate concerns strictly, avoid premature abstraction, measure timing accurately, document unit conventions explicitly, and never leave unexplained "magic numbers".

## Design Patterns Summary

### Architectural Patterns
- **Producer-Consumer:** Decoupled timing layers (1ms Harvester → 5ms Refiner → 10ms Navigator) prevent priority inversion.
- **Singleton:** SensorRegistry ensures single source of truth for sensor data; CalibrationEngine manages global calibration state.
- **State Machine:** Each Calibrator implements a state machine for interactive, guided calibration sequences.
- **Template Method:** CalibratorBase defines calibration workflow; subclasses implement sensor-specific logic.

### Concurrency Patterns
- **Lock-free Ring Buffer:** CircularBuffer uses atomic indices for 1ms→5ms handoff without spinning threads.
- **Atomic Telemetry:** Jitter stats tracked with `std::atomic<long long>` to avoid blocking high-priority threads.
- **Thread Priority Segregation:** Harvester (SCHED_FIFO) isolated from Navigator (SCHED_OTHER) to prevent starvation.

### Real-Time Patterns
- **Absolute Timing:** `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` corrects for execution jitter, ensuring periodic beats.
- **RAII Timing:** Cycle timing variables initialized at thread creation, preventing late allocations.

## Current Project Phase

### Completed
- Hardware abstraction (I2cMaster, LSM9DS1 dual-node architecture)
- 1ms/5ms/10ms thread scheduling with SCHED_FIFO/SCHED_OTHER priorities
- Jitter telemetry and OS interference detection
- 9-DOF initialization (Accel/Gyro/Mag + Pressure/Temperature)
- Producer-Consumer data pipeline with CircularBuffer and SensorRegistry
- Hard-iron magnetometer calibration (min-max method)

### In Progress / Next Steps
- **Immediate:** Implement calibration framework with state machines (CalibrationEngine, Calibrator classes)
- **Short-term:** Calibration file I/O and persistence (CalibDataStore)
- **Medium-term:** AHRS (Attitude & Heading Reference System) for Pitch/Roll/Yaw calculation
- **Long-term:** IPC broadcast of telemetry to external monitors + Yocto kernel core isolation (Scenario C)
