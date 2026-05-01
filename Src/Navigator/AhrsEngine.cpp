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
                                     bool& magValid, float& beta) const
    {
        accelMag = m_lastAccelMag;
        magMag   = m_lastMagMag;
        magValid = m_lastMagValid;
        beta     = m_lastBeta;
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

        /* 4. Beta selection: warmup + dynamic motion detection.
         * During WARMUP_CYCLES a higher beta snaps the quaternion from identity to the
         * true orientation in the first few cycles instead of drifting in slowly.
         * Accel values are in m/s² (the calibration pipeline preserves SI units).
         * At rest (any static tilt), |a| == 9.81 m/s² exactly.
         * During movement, linear accelerations push |a| away from GRAVITY_MS2. Driving
         * beta to 0 converts the update to pure gyro integration for that cycle,
         * preventing the linear-acceleration artifact from corrupting pitch and roll. */
        float baseBeta  = (m_cycleCount < WARMUP_CYCLES) ? AHRS_WARMUP_BETA : AHRS_MADGWICK_BETA;
        float accelMag  = sqrtf(ax*ax + ay*ay + az*az);
        float effectiveBeta = (fabsf(accelMag - AHRS_GRAVITY_MS2) < AHRS_MOTION_THRESHOLD)
                            ? baseBeta
                            : 0.0f;
        m_filter.setBeta(effectiveBeta);

        /* 5. Magnetometer quality gate.
         * Earth's field: 0.25–0.65 G (Gauss matches LSM9DS1 output units).
         * The 9-DOF gradient mixes magnetic and gravity terms into all quaternion
         * components, so a badly calibrated magnetometer corrupts pitch and roll too —
         * not just yaw. Fall back to 6-DOF (accel + gyro only) when |mag| is outside
         * the plausible Earth-field range. */
        float magMag  = sqrtf(mx*mx + my*my + mz*mz);
        bool  magValid = (magMag >= AHRS_MAG_NORM_MIN && magMag <= AHRS_MAG_NORM_MAX);
        if (magValid) {
            m_filter.update(m_q, ax, ay, az, gx, gy, gz, mx, my, mz, dt);
        } else {
            m_filter.updateIMU(m_q, ax, ay, az, gx, gy, gz, dt);
        }

        /* 6. Store diagnostics for the dashboard */
        m_lastAccelMag  = accelMag;
        m_lastMagMag    = magMag;
        m_lastMagValid  = magValid;
        m_lastBeta      = effectiveBeta;

        /* 7. Convert quaternion to Euler angles */
        float roll, pitch, yaw;
        quaternionToEuler(roll, pitch, yaw);

        /* 8. Advance cycle counter and determine warm-up state */
        m_cycleCount++;
        bool ready = (m_cycleCount >= WARMUP_CYCLES);

        /* 9. Store orientation in SensorsRegistry */
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
