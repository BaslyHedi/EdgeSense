# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Configure
cmake --preset rpi5          # Raspberry Pi 5 (aarch64 cross-compile → build/rpi5/)
cmake --preset host          # Local Ubuntu x86_64 (→ build/host/)

# Build
cmake --build build/rpi5
cmake --build build/host

# Deploy to RPi5
scp build/rpi5/EdgeSenseApp HediPi:/home/hedi/EdgeSenseApp

# Run (on device)
./EdgeSenseApp               # Application mode
./EdgeSenseApp --calib       # Calibration mode
```

## Architecture

Three-tier producer-consumer pipeline — each tier runs in its own thread:

| Tier | Period | Scheduler | Role |
|------|--------|-----------|------|
| Harvester | 5ms | SCHED_FIFO prio 95 | Raw I2C reads → CircularBuffer |
| Refiner | 25ms | SCHED_FIFO prio 80 | Average/filter → filtered snapshots |
| Process | 50ms | SCHED_OTHER | Fusion, logging, telemetry |

Dual execution modes: **APP** (all 3 tiers) and **CALIB** (Harvester + Process only).

Data flows: sensors → `SensorsRegistry` (CircularBuffer<T,50> raw) → Refiner → mutex-protected filtered snapshots → Process tier.

### Key Classes

- `ThreadManager` — owns 3 threads, schedules via `clock_nanosleep(TIMER_ABSTIME)`, injects `std::function<void()>` tasks per mode
- `SensorManager` — sensor lifecycle, sets up APP/CALIB task sets, mode switching
- `SensorsRegistry` (Singleton) — centralized data nexus; lock-free raw buffers + mutex snapshots
- `CalibrationEngine` (Singleton) — orchestrates Accel → Gyro → Mag → PressTemp calibration sequence
- `CalibratorBase` — abstract state machine; subclasses implement `startCalibration()`, `processCalibration()`, `isComplete()`
- `CalibDataStore` — binary + JSON persistence (`./CalibData/SensorsCalib.bin` + `.json`)
- `I2cMaster` — RAII Linux `/dev/i2c-1` wrapper; shared by all sensor drivers

### Sensor Hardware

| Sensor | I2C Address | Data |
|--------|-------------|------|
| LSM9DS1_AccGyro | 0x6A | Accel ±2g, Gyro 245°/s at 119Hz |
| LSM9DS1_Mag | 0x1E | Magnetometer ±4G at 80Hz |
| LPS25HB | 0x5C | Pressure 260-1260hPa, Temp |

## Code Rules (Non-Negotiable)

1. **Comments:** Use `/* */` only. Never use `//` comments anywhere.
2. **Single exit:** Every function has exactly one `return` statement at the end. Use a `retVal` local variable for branches.
3. **RAII:** All resources (file descriptors, mutexes, threads) are tied to object lifetimes. Never manually release in non-destructor paths.
4. **No I/O in hot loops:** No `std::cout`, file writes, or blocking I/O inside Harvester or Refiner tier callbacks, or inside the APP-mode Process tier callback. Route all output through `LOG_INFO()`/`LOG_WARN()`/`LOG_ERROR()` macros which dispatch to an async worker thread.

## Collaboration Style

This is a **learning project**. Hedi makes all architectural decisions. When multiple approaches exist, present the trade-offs and let him choose — never impose a solution. Explain the *why* behind best practices.
