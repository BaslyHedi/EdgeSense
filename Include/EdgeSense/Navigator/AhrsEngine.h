/**
 * @file AhrsEngine.h
 * @author Hedi Basly
 * @brief Singleton AHRS orchestrator: reads calibrated sensor snapshots,
 *        drives the Madgwick filter, and stores orientation in SensorsRegistry.
 * @date 2026-04-26
 *
 * Unit conversion responsibilities (documented here to be explicit):
 *   Gyroscope    : °/s  →  rad/s  (× π/180) — applied before calling MadgwickFilter.
 *                  The registry stores °/s because CalibrationEngine subtracts gyro
 *                  bias in °/s. Madgwick requires rad/s.
 *   Accelerometer: values are in m/s² (0.000061 g/LSB × 9.80665 m/s²/g × raw).
 *                  MadgwickFilter normalises internally so the unit does not affect
 *                  pitch/roll direction. The 9.81 m/s² magnitude is used for the
 *                  motion detection threshold (AHRS_GRAVITY_MS2).
 *   Magnetometer : raw mG passed through — MadgwickFilter normalises internally,
 *                  so the absolute unit is irrelevant.
 *
 * Magnetic declination: applied as yaw += declination_deg after Euler conversion.
 * Configure via setDeclination(). Default 0.0f (no correction).
 *
 * Warm-up: first WARMUP_CYCLES calls produce valid=false in SensorsRegistry.
 * The filter converges from identity quaternion [1,0,0,0] toward true orientation.
 * Consumers must check valid before using roll/pitch/yaw values.
 */

#pragma once
#include <chrono>
#include <EdgeSense/Navigator/MadgwickFilter.h>
#include <EdgeSense/Navigator/OrientationData.h>

/* Madgwick gradient-descent gain.
 * Higher = stronger accel/mag correction, faster convergence, more noise sensitivity.
 * Lower  = smoother output, slower correction, more gyro drift.
 * Tune here only. Typical range [0.1, 1.5]. */
#define AHRS_MADGWICK_BETA 1.0f

/* Standard gravity (m/s²). The LSM9DS1 accel outputs m/s² (0.000061 g/LSB × 9.80665).
 * Used as the rest reference for the motion detection gate below. */
#define AHRS_GRAVITY_MS2 9.80665f

/* Dynamic motion rejection threshold (m/s²).
 * The accel pipeline produces values in m/s², so the rest reference is ~9.81 m/s², not 1.
 * When |accel| deviates from AHRS_GRAVITY_MS2 by more than this amount, linear
 * accelerations are corrupting the gravity reference vector. Accel correction is
 * disabled for that cycle (beta forced to 0) and the filter falls back to pure gyro
 * integration. 0.981 m/s² == 0.1 g — tight but practical for a desk-mounted board. */
#define AHRS_MOTION_THRESHOLD 0.981f

/* Beta used during the WARMUP_CYCLES period only.
 * Must be high enough to converge yaw from 0° to true magnetic heading within
 * WARMUP_CYCLES steps, but kept below ~1.5 to avoid period-2 oscillation in
 * pitch/roll caused by the coupled magnetometer gradient during yaw convergence. */
#define AHRS_WARMUP_BETA 1.0f

/* Gyroscope rotation-rate threshold for adaptive beta scaling (rad/s).
 * When |ω| is small (near-static board), the accel correction reference is reliable
 * and beta can be applied at full strength. When |ω| exceeds this threshold the board
 * is rotating, and the accel reading is increasingly delayed relative to true gravity
 * direction — the correction starts fighting the rotation instead of correcting drift.
 *
 * Above the threshold, effectiveBeta is scaled down as:
 *   beta_effective = beta × (GYRO_THRESHOLD / |ω|)
 * This keeps the accel contribution smaller than the gyro integration, allowing the
 * filter to track the rotation instead of anchoring to the previous "flat" orientation.
 *
 * 0.1 rad/s ≈ 5.7 °/s — well above the typical calibrated gyro residual (< 0.5°/s)
 * and well below any deliberate hand rotation (typically 30–300 °/s). */
#define AHRS_GYRO_THRESHOLD_RADS 0.1f

/* Magnetometer norm sanity gate (Gauss — matches LSM9DS1 output units: 0.00014 G/LSB).
 * Earth's field ranges ~0.25–0.65 G depending on location.
 * Outside [MIN, MAX] signals uncalibrated, saturated, or interfered magnetometer data.
 * When the gate fails the 9-DOF update is replaced with a 6-DOF accel+gyro update so
 * that bad magnetic data cannot corrupt pitch and roll via the shared gradient terms. */
#define AHRS_MAG_NORM_MIN  0.1f   /* Gauss */
#define AHRS_MAG_NORM_MAX  1.2f   /* Gauss */

/* Gravity alignment gate (degrees).
 * The 9-DOF Madgwick gradient couples the magnetic and gravity terms. When roll/pitch
 * error is large (>20°), the magnetic gradient partially opposes gravity correction,
 * reducing effective convergence rate by up to 6×. Force 6-DOF until the gravity
 * vector is aligned, then re-enable magnetometer for yaw tracking only. */
#define AHRS_GRAVITY_CONVERGENCE_THRESHOLD_DEG 20.0f

namespace EdgeSense {
    namespace Navigator {

    class AhrsEngine {
    public:

        static AhrsEngine& getInstance() {
            static AhrsEngine instance;
            return instance;
        }

        AhrsEngine(const AhrsEngine&)            = delete;
        AhrsEngine& operator=(const AhrsEngine&) = delete;

        /*
         * Run one AHRS update cycle.
         * Called by SensorManager::appProcessAction() at the PROCESS tier rate (50ms).
         * Reads calibrated snapshots from SensorsRegistry, calls MadgwickFilter::update(),
         * converts quaternion to Euler angles, and writes results back to SensorsRegistry.
         */
        void update();

        /* Magnetic declination correction applied to yaw output (degrees).
         * Positive = East declination (e.g. +2.5 for Paris). */
        void setDeclination(float degrees);

        /* Adjust Madgwick beta gain at runtime. */
        void setBeta(float beta);

        /* True once the warm-up period has passed. */
        bool isReady() const;

        /* Last-cycle diagnostics — read from the dashboard thread after update().
         * accelMag    : |accel| in g   (should be ~1.0 at rest; deviation triggers motion gate)
         * magMag      : |mag|   in G   (should be 0.25–0.65 G; outside range → 6-DOF fallback)
         * magValid    : true = 9-DOF used; false = fell back to 6-DOF this cycle
         * beta        : effective Madgwick beta actually applied this cycle
         * gyroMag     : |ω| in rad/s  (above AHRS_GYRO_THRESHOLD_RADS → beta was scaled down)
         * gravityError: angle (°) between filter's gravity estimate and measured accel;
         *               >20° → gravity gate active, mag gradient bypassed for fast catch-up */
        void getDiagnostics(float& accelMag, float& magMag, bool& magValid,
                             float& beta, float& gyroMag, float& gravityError) const;

    private:

        AhrsEngine();

        MadgwickFilter m_filter;
        Quaternion     m_q;
        float          m_declination_deg;
        int            m_cycleCount;

        std::chrono::steady_clock::time_point m_lastUpdate;
        bool  m_firstCall;

        /* Diagnostic state written each update() cycle, read by getDiagnostics() */
        float m_lastAccelMag     = 1.0f;
        float m_lastMagMag       = 0.0f;
        bool  m_lastMagValid     = false;
        float m_lastBeta         = 0.0f;
        float m_lastGyroMag      = 0.0f; /* rad/s — shows how much the gyro scaling reduced beta */
        float m_lastGravityError = 0.0f; /* degrees — >20° means gravity gate active (6-DOF) */

        /* 200 cycles × 10ms = 2 seconds warm-up */
        static constexpr int WARMUP_CYCLES = 200;

        static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

        /*
         * Convert the current quaternion m_q to roll/pitch/yaw in degrees.
         * Applies LSM9DS1 coordinate frame (X=fwd, Y=left, Z=up) ZYX convention.
         * Adds m_declination_deg to yaw.
         */
        void quaternionToEuler(float& roll_deg, float& pitch_deg, float& yaw_deg) const;
    };

    } /* namespace Navigator */
} /* namespace EdgeSense */
