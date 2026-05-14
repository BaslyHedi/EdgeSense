/**
 * @file AhrsEngine.cpp
 * @author Hedi Basly
 * @brief Implementation of AhrsEngine singleton
 * @date 2026-04-26
 */

#include <EdgeSense/Navigator/AhrsEngine.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Logger/Logger.h>
#include <cmath>
#include <iostream>

using namespace EdgeSense::Logger;
using namespace EdgeSense::Sensors;

namespace EdgeSense {
    namespace Navigator {

    AhrsEngine::AhrsEngine()
        : m_filter(AHRS_MADGWICK_BETA),
          m_q(),
          m_declination_deg(0.0f),
          m_cycleCount(0),
          m_firstCall(true)
    {
        std::string banner = "[AHRS] AhrsEngine initialised (Madgwick beta="
                           + std::to_string(AHRS_MADGWICK_BETA)
                           + ", warmup=" + std::to_string(WARMUP_CYCLES) + " cycles)";
        LOG_INFO(banner);
    }

    void AhrsEngine::setDeclination(float degrees) { m_declination_deg = degrees; }
    void AhrsEngine::setBeta(float beta)            { m_filter.setBeta(beta); }
    bool AhrsEngine::isReady() const                { return (m_cycleCount >= WARMUP_CYCLES); }

    void AhrsEngine::getDiagnostics(float& accelMag, float& magMag,
                                     bool& magValid, float& beta,
                                     float& gyroMag, float& gravityError) const
    {
        accelMag     = m_lastAccelMag;
        magMag       = m_lastMagMag;
        magValid     = m_lastMagValid;
        beta         = m_lastBeta;
        gyroMag      = m_lastGyroMag;
        gravityError = m_lastGravityError;
    }

    void AhrsEngine::update() {
        /* 1. Measure actual dt with steady_clock — never assume a fixed 50ms */
        auto now = std::chrono::steady_clock::now();
        float dt;
        if (m_firstCall) {
            dt = 0.05f; /* safe default for first cycle only */
            m_firstCall = false;
        } else {
            dt = std::chrono::duration<float>(now - m_lastUpdate).count();
            /* Clamp dt to [1ms, 200ms] — guards against debug pauses or OS suspend */
            if (dt < 0.001f) { dt = 0.001f; }
            if (dt > 0.200f) { dt = 0.200f; }
        }
        m_lastUpdate = now;

        /* 2. Read calibrated sensor snapshots from SensorsRegistry */
        auto& registry = SensorsRegistry::getInstance();
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        registry.getFilteredImuAccel(ax, ay, az);
        registry.getFilteredImuGyro(gx, gy, gz);
        registry.getFilteredImuMag(mx, my, mz);

        /* 3. Unit conversion: gyro °/s → rad/s (see header for rationale) */
        gx *= DEG_TO_RAD;
        gy *= DEG_TO_RAD;
        gz *= DEG_TO_RAD;

        /* 4. Beta selection: warmup + accel gate + gyro-magnitude scaling.
         *
         * baseBeta: higher during WARMUP_CYCLES so yaw converges to magnetic north before
         * the valid flag is asserted; drops to AHRS_MADGWICK_BETA afterward.
         *
         * Accel gate: if |accel| deviates from 9.81 m/s² by more than MOTION_THRESHOLD,
         * a significant linear acceleration is present. Set beta = 0 (pure gyro) to
         * prevent the corrupted gravity reference from twisting pitch and roll.
         *
         * Gyro scaling: even without a large accel deviation, a fast rotation means the
         * accel reading is delayed relative to the true gravity direction. The correction
         * then actively fights the rotation and clamps the filter to the previous orientation
         * (observed: board physically at -90° but filter only showed -24°). Scale beta
         * down proportionally to |ω| so the gyro integration dominates during rotation:
         *   beta_effective = baseBeta × min(1, GYRO_THRESHOLD / |ω|)
         * At rest (|ω| < threshold): no reduction. At 49°/s (0.86 rad/s): ~12× reduction. */
        float baseBeta  = (m_cycleCount < WARMUP_CYCLES) ? AHRS_WARMUP_BETA : AHRS_MADGWICK_BETA;
        float accelMag  = sqrtf(ax*ax + ay*ay + az*az);
        float gyroMag   = sqrtf(gx*gx + gy*gy + gz*gz); /* rad/s after step 3 conversion */
        float gyroFactor = (gyroMag > AHRS_GYRO_THRESHOLD_RADS)
                         ? (AHRS_GYRO_THRESHOLD_RADS / gyroMag)
                         : 1.0f;
        float effectiveBeta = (fabsf(accelMag - AHRS_GRAVITY_MS2) < AHRS_MOTION_THRESHOLD)
                            ? baseBeta * gyroFactor
                            : 0.0f;
        m_filter.setBeta(effectiveBeta);

        /* 5. Gravity alignment gate.
         * The Madgwick 9-DOF gradient couples magnetic and gravity corrections. When roll/pitch
         * error is large, the magnetic gradient partially opposes the gravity gradient, reducing
         * effective convergence by up to 6× (observed: 12s instead of 2s from 143° error).
         *
         * Estimated body-frame gravity direction from current quaternion (Madgwick eq. 25):
         *   g_est = [2(qx·qz − qw·qy),  2(qw·qx + qy·qz),  1 − 2qx² − 2qy²]
         *
         * When the angle between g_est and the normalised measured accel exceeds
         * AHRS_GRAVITY_CONVERGENCE_THRESHOLD_DEG, bypass the magnetometer and run 6-DOF
         * so gravity converges at full speed. Re-enable 9-DOF once gravity is aligned. */
        float g_est_x = 2.0f*(m_q.x*m_q.z - m_q.w*m_q.y);
        float g_est_y = 2.0f*(m_q.w*m_q.x + m_q.y*m_q.z);
        float g_est_z = 1.0f - 2.0f*m_q.x*m_q.x - 2.0f*m_q.y*m_q.y;
        float ax_n = ax / accelMag, ay_n = ay / accelMag, az_n = az / accelMag;
        float dot  = g_est_x*ax_n + g_est_y*ay_n + g_est_z*az_n;
        if (dot >  1.0f) { dot =  1.0f; }
        if (dot < -1.0f) { dot = -1.0f; }
        float gravityError_deg = RAD_TO_DEG * acosf(dot);
        bool  gravityAligned   = (gravityError_deg < AHRS_GRAVITY_CONVERGENCE_THRESHOLD_DEG);

        /* 6. Magnetometer quality gate (only active when gravity is aligned).
         * Earth's field: 0.25–0.65 G (Gauss matches LSM9DS1 output units).
         * The 9-DOF gradient mixes magnetic and gravity terms into all quaternion
         * components, so a badly calibrated magnetometer corrupts pitch and roll too —
         * not just yaw. Fall back to 6-DOF when |mag| is outside the plausible range
         * OR when gravity is not yet aligned (gravity gate takes priority). */
        float magMag   = sqrtf(mx*mx + my*my + mz*mz);
        bool  magValid = gravityAligned &&
                         (magMag >= AHRS_MAG_NORM_MIN && magMag <= AHRS_MAG_NORM_MAX);
        if (magValid) {
            m_filter.update(m_q, ax, ay, az, gx, gy, gz, mx, my, mz, dt);
        } else {
            m_filter.updateIMU(m_q, ax, ay, az, gx, gy, gz, dt);
        }

        /* 7. Store diagnostics for the dashboard */
        m_lastAccelMag     = accelMag;
        m_lastMagMag       = magMag;
        m_lastMagValid     = magValid;
        m_lastBeta         = effectiveBeta;
        m_lastGyroMag      = gyroMag;
        m_lastGravityError = gravityError_deg;

        /* 8. Convert quaternion to Euler angles */
        float roll, pitch, yaw;
        quaternionToEuler(roll, pitch, yaw);

        /* 9. Advance cycle counter and determine warm-up state */
        m_cycleCount++;
        bool ready = (m_cycleCount >= WARMUP_CYCLES);

        /* 10. Store orientation in SensorsRegistry */
        registry.updateOrientation(roll, pitch, yaw, ready);
    }

    void AhrsEngine::quaternionToEuler(float& roll_deg,
                                        float& pitch_deg,
                                        float& yaw_deg) const
    {
        /* ZYX Euler extraction from unit quaternion.
         * Coordinate frame: LSM9DS1 default (X=forward, Y=left, Z=up).
         *
         * Roll  (X): atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
         * Pitch (Y): asin(clamp(2(q0q2 - q3q1), -1, 1))
         * Yaw   (Z): atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²)) + declination
         */
        float q0 = m_q.w, q1 = m_q.x, q2 = m_q.y, q3 = m_q.z;

        roll_deg = RAD_TO_DEG * atan2f(2.0f*(q0*q1 + q2*q3),
                                        1.0f - 2.0f*(q1*q1 + q2*q2));

        float sinp = 2.0f*(q0*q2 - q3*q1);
        if (sinp >  1.0f) { sinp =  1.0f; } /* clamp against floating-point overshoot */
        if (sinp < -1.0f) { sinp = -1.0f; }
        pitch_deg = RAD_TO_DEG * asinf(sinp);

        float raw_yaw = RAD_TO_DEG * atan2f(2.0f*(q0*q3 + q1*q2),
                                              1.0f - 2.0f*(q2*q2 + q3*q3));
        yaw_deg = raw_yaw + m_declination_deg;
    }

    } /* namespace Navigator */
} /* namespace EdgeSense */
