/**
 * @file AhrsEngine.h
 * @author Hedi Basly
 * @brief Singleton AHRS orchestrator: reads calibrated sensor snapshots,
 *        drives the Madgwick filter, and stores orientation in SensorsRegistry.
 * @date 2026-04-26
 *
 * Unit conversion responsibilities:
 *   Gyroscope    : °/s  →  rad/s  (× π/180) — applied before MadgwickFilter.
 *   Accelerometer: m/s² — normalised internally by the filter.
 *   Magnetometer : raw mG — normalised internally by the filter.
 *
 * Magnetic declination: applied as yaw += declination_deg after Euler conversion.
 *
 * Initialisation: on the first update() call the quaternion is initialised from
 * the live accel+mag reading (tilt-compensated heading) instead of identity.
 * This eliminates the long warm-up period and makes valid=true from the first cycle.
 */

#pragma once
#include <chrono>
#include <EdgeSense/Navigator/MadgwickFilter.h>
#include <EdgeSense/Navigator/OrientationData.h>

/* Madgwick β: gradient-descent gain for accel/mag correction.
 * Derived from gyro noise density: β = √(3/4)·σ_gyro.
 * For LSM9DS1 at 119 Hz (noise ≈ 0.007 dps/√Hz): β ≈ 0.001.
 *
 * β must satisfy: β × sin(ε) × fs > gz_residual  to prevent Roll divergence in
 * 9-DOF mode.  At ε=1°, fs=20Hz: β × 0.0175 × 20 > gz_residual.
 * With observed gz_residual ≈ 0.006 rad/s: β > 0.017 required.
 * β=0.025 gives correction rate 0.0088 rad/s — 47% margin over gz_residual.
 *
 * Yaw drift at β=0.025 ≈ 0.33°/s (vs 1.88°/s at β=0.033) — acceptable trade-off.
 * Once gyro is recalibrated at thermal steady-state (gz_residual < 0.001 rad/s),
 * β can be reduced to 0.01 to minimise yaw correction from mag residuals.
 *
 * Must NOT be tuned upward to compensate for missing gyro bias estimation —
 * use AHRS_MADGWICK_ZETA for that instead. */
#define AHRS_MADGWICK_BETA  0.025f

/* Madgwick ζ: gyro bias estimation rate (Eq. 47–49 of Madgwick 2010).
 * Integrates the body-frame angular error into a running bias correction so
 * that residual calibration error does not accumulate over time.
 * Lower ζ → slower convergence but monotonic (no overshoot/oscillation).
 * Higher ζ → faster convergence but risks oscillation when residual bias is
 * large (>0.5°/s, e.g. from thermal drift). At ζ=0.01 the maximum convergence
 * rate is 2·ζ·dt = 0.001 rad/s per cycle, reaching 0.03 rad/s bias in ~30 cycles. */
#define AHRS_MADGWICK_ZETA  0.01f

/* Standard gravity reference (m/s²). */
#define AHRS_GRAVITY_MS2    9.80665f

/* Linear-acceleration gate (m/s²).
 * When |accel| deviates from gravity by more than this, the board has
 * significant linear acceleration. Gravity reference is unreliable: skip
 * both accel and mag corrections for this cycle. 0.981 m/s² ≈ 0.1 g. */
#define AHRS_MOTION_THRESHOLD 0.981f

/* Magnetometer norm sanity gate (Gauss — matches LSM9DS1 output units). */
#define AHRS_MAG_NORM_MIN   0.1f
#define AHRS_MAG_NORM_MAX   1.2f

/* Gravity alignment gate (degrees).
 * When ge exceeds this threshold the 9-DOF magnetic gradient cross-terms create
 * a POSITIVE FEEDBACK loop: Pitch/Roll error grows the cross-term, which grows
 * Pitch/Roll further, with a time constant of ~7 s. At 20° the system reaches
 * full divergence within 30 s. At 8° the feedback has not yet gained traction:
 *   - gate fires within ~25 s of startup (gz_residual ≈ 0.3°/s case)
 *   - 6-DOF recovery from 8°: beta × sin(8°) × 20 Hz = 0.070 rad/s >> gz, ~2 s
 *   - prevents catastrophic quaternion divergence past 90° pitch
 * If gz_residual is calibrated to < 0.1°/s this threshold can be raised back
 * to 20° without risk of divergence. */
#define AHRS_GRAVITY_CONVERGENCE_THRESHOLD_DEG  8.0f

/* Magnetic disturbance gate (degrees).
 * Maximum allowed angle between the current measured earth-frame field and the
 * stored reference field. Ferrous objects and motors change field direction
 * without necessarily changing its magnitude — the norm gate alone misses these.
 * 15° is a practical threshold: tight enough to reject hard-iron spikes,
 * loose enough to handle sensor noise. */
#define AHRS_MAG_DISTURBANCE_DEG  15.0f

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

        /* Run one AHRS update cycle (called at the PROCESS tier rate, 50 ms). */
        void update();

        /* Magnetic declination correction added to yaw output (degrees).
         * Positive = East declination. */
        void setDeclination(float degrees);

        /* Adjust Madgwick β at runtime (for tuning only). */
        void setBeta(float beta);

        /* True after the first successful initialisation from sensor data. */
        bool isReady() const;

        /* Last-cycle diagnostics.
         * accelMag    : |accel| m/s²  (~9.81 at rest)
         * magMag      : |mag|   Gauss (~0.25–0.65)
         * magValid    : true = 9-DOF used this cycle
         * beta        : effective β applied this cycle
         * gyroMag     : |ω| rad/s
         * gravityError: angle (°) between filter gravity estimate and measured accel */
        void getDiagnostics(float& accelMag, float& magMag, bool& magValid,
                             float& beta, float& gyroMag, float& gravityError) const;

    private:

        AhrsEngine();

        /* Initialise quaternion analytically from first accel+mag reading.
         * Computes roll/pitch from gravity tilt, yaw from tilt-compensated heading.
         * Also seeds the magnetic field reference for the disturbance gate. */
        void initFromSensors(float ax, float ay, float az,
                              float mx, float my, float mz);

        /* Check whether the current magnetometer reading is consistent with the
         * stored reference field direction. Returns false if the angle exceeds
         * AHRS_MAG_DISTURBANCE_DEG (magnetic disturbance detected). */
        bool magUndisturbed(float mx, float my, float mz) const;

        /* Update the stored magnetic reference from an accepted earth-frame field
         * vector [hx, hy, hz]. Uses a slow EMA to track genuine field changes. */
        void updateMagReference(float hx, float hy, float hz);

        /* ZYX Euler extraction from m_q. Handles the ±90° pitch singularity with
         * the degenerate-case formula (Roll = 0, Yaw absorbs heading). */
        void quaternionToEuler(float& roll_deg, float& pitch_deg, float& yaw_deg) const;

        MadgwickFilter m_filter;
        Quaternion     m_q;
        float          m_declination_deg;
        int            m_cycleCount;

        std::chrono::steady_clock::time_point m_lastUpdate;
        bool  m_firstCall;

        /* Stored magnetic reference field (earth frame, normalised).
         * [m_refBx, 0, m_refBz] — hy component is zero by construction (Eq. 45). */
        float m_refBx      = 0.0f;
        float m_refBz      = 0.0f;
        bool  m_refInitialized = false;

        float m_lastAccelMag     = 1.0f;
        float m_lastMagMag       = 0.0f;
        bool  m_lastMagValid     = false;
        float m_lastBeta         = 0.0f;
        float m_lastGyroMag      = 0.0f;
        float m_lastGravityError = 0.0f;

        /* First update() call is valid output; no warm-up delay with proper init. */
        static constexpr int WARMUP_CYCLES = 1;

        /* Mandatory 6-DOF startup: ζ converges gx/gy from gravity before the
         * 9-DOF mag Jacobian s1 cross-term can amplify Roll. At 50 ms/cycle
         * this is 2 seconds; Yaw drift over startup ≈ gz_bias × 2 s ≈ 1.7°. */
        static constexpr int STARTUP_6DOF_CYCLES = 40;

        static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
    };

    } /* namespace Navigator */
} /* namespace EdgeSense */
