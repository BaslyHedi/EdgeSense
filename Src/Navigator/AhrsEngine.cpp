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
        : m_filter(AHRS_MADGWICK_BETA, AHRS_MADGWICK_ZETA),
          m_q(),
          m_declination_deg(0.0f),
          m_cycleCount(0),
          m_firstCall(true)
    {
        std::string banner = "[AHRS] AhrsEngine initialised (Madgwick beta="
                           + std::to_string(AHRS_MADGWICK_BETA)
                           + " zeta=" + std::to_string(AHRS_MADGWICK_ZETA) + ")";
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

    void AhrsEngine::initFromSensors(float ax, float ay, float az,
                                      float mx, float my, float mz)
    {
        /* Compute initial attitude from static accel + tilt-compensated mag heading.
         * This seeds the quaternion close to reality so the filter converges immediately
         * instead of from identity [1,0,0,0] over hundreds of cycles.
         *
         * Roll and pitch from gravity (assumes board is near-static at startup):
         *   roll  = atan2(ay, az)
         *   pitch = atan2(-ax, sqrt(ay²+az²))
         *
         * Tilt-compensated magnetic heading (LSM9DS1: X=fwd, Y=left, Z=up):
         *   mx_h =  mx·cos(pitch) + mz·sin(pitch)
         *   my_h =  mx·sin(roll)·sin(pitch) + my·cos(roll) − mz·sin(roll)·cos(pitch)
         *   yaw  =  atan2(−my_h, mx_h)
         *
         * ZYX quaternion from roll/pitch/yaw (half-angle products):
         *   q = Rz(yaw) ⊗ Ry(pitch) ⊗ Rx(roll)
         */
        float roll  = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

        float sinR = sinf(roll),  cosR = cosf(roll);
        float sinP = sinf(pitch), cosP = cosf(pitch);

        float mx_h =  mx * cosP + mz * sinP;
        float my_h =  mx * sinR * sinP + my * cosR - mz * sinR * cosP;
        float yaw   = atan2f(-my_h, mx_h);

        float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
        float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);

        m_q.w =  cy*cp*cr + sy*sp*sr;
        m_q.x =  cy*cp*sr - sy*sp*cr;
        m_q.y =  cy*sp*cr + sy*cp*sr;
        m_q.z =  sy*cp*cr - cy*sp*sr;

        /* Seed the magnetic reference field from the same reading.
         * Earth-frame field: rotate [mx,my,mz] with the freshly computed q. */
        float q0 = m_q.w, q1 = m_q.x, q2 = m_q.y, q3 = m_q.z;
        float magNorm = sqrtf(mx*mx + my*my + mz*mz);
        if (magNorm > 0.0f) {
            float rN = 1.0f / magNorm;
            float mnx = mx*rN, mny = my*rN, mnz = mz*rN;
            float hx = 2.0f*(mnx*(0.5f - q2*q2 - q3*q3) + mny*(q1*q2 - q0*q3) + mnz*(q1*q3 + q0*q2));
            float hy = 2.0f*(mnx*(q1*q2 + q0*q3) + mny*(0.5f - q1*q1 - q3*q3) + mnz*(q2*q3 - q0*q1));
            float hz = 2.0f*(mnx*(q1*q3 - q0*q2) + mny*(q2*q3 + q0*q1) + mnz*(0.5f - q1*q1 - q2*q2));
            m_refBx = sqrtf(hx*hx + hy*hy);
            m_refBz = hz;
            m_refInitialized = true;
        }
    }

    bool AhrsEngine::magUndisturbed(float mx, float my, float mz) const
    {
        /* Returns false when the measured field direction deviates from the stored
         * reference by more than AHRS_MAG_DISTURBANCE_DEG. Magnitude-only checks
         * (norm gate) miss disturbances that rotate the field without scaling it. */
        bool result = true;
        if (m_refInitialized) {
            float q0 = m_q.w, q1 = m_q.x, q2 = m_q.y, q3 = m_q.z;
            float magNorm = sqrtf(mx*mx + my*my + mz*mz);
            if (magNorm > 0.0f) {
                float rN = 1.0f / magNorm;
                float mnx = mx*rN, mny = my*rN, mnz = mz*rN;

                /* Rotate body-frame mag to earth frame */
                float hx = 2.0f*(mnx*(0.5f - q2*q2 - q3*q3) + mny*(q1*q2 - q0*q3) + mnz*(q1*q3 + q0*q2));
                float hy = 2.0f*(mnx*(q1*q2 + q0*q3) + mny*(0.5f - q1*q1 - q3*q3) + mnz*(q2*q3 - q0*q1));
                float hz = 2.0f*(mnx*(q1*q3 - q0*q2) + mny*(q2*q3 + q0*q1) + mnz*(0.5f - q1*q1 - q2*q2));

                /* Angle between measured [hx,hy,hz] and reference [m_refBx, 0, m_refBz].
                 * Both are already normalised (body mag was normalised above). */
                float dot = hx*m_refBx + hz*m_refBz; /* hy component of reference is 0 */
                float refNorm = sqrtf(m_refBx*m_refBx + m_refBz*m_refBz);
                float measNorm = sqrtf(hx*hx + hy*hy + hz*hz);
                if (refNorm > 0.0f && measNorm > 0.0f) {
                    float cosAngle = dot / (refNorm * measNorm);
                    if (cosAngle >  1.0f) { cosAngle =  1.0f; }
                    if (cosAngle < -1.0f) { cosAngle = -1.0f; }
                    float angleDeg = RAD_TO_DEG * acosf(cosAngle);
                    result = (angleDeg < AHRS_MAG_DISTURBANCE_DEG);
                }
            }
        }
        return result;
    }

    void AhrsEngine::updateMagReference(float hx, float hy, float hz)
    {
        /* Slow exponential moving average to track genuine long-term field changes
         * (e.g. moving to a different room) without chasing transient disturbances.
         * Alpha = 0.01 → time constant ≈ 100 cycles ≈ 5 seconds at 50 ms/cycle. */
        static constexpr float alpha = 0.01f;
        float newBx = sqrtf(hx*hx + hy*hy);
        float newBz = hz;
        m_refBx = (1.0f - alpha) * m_refBx + alpha * newBx;
        m_refBz = (1.0f - alpha) * m_refBz + alpha * newBz;
    }

    void AhrsEngine::update() {
        /* 1. Measure actual dt */
        auto now = std::chrono::steady_clock::now();
        float dt;
        if (m_firstCall) {
            dt = 0.05f;
        } else {
            dt = std::chrono::duration<float>(now - m_lastUpdate).count();
            if (dt < 0.001f) { dt = 0.001f; }
            if (dt > 0.200f) { dt = 0.200f; }
        }
        m_lastUpdate = now;

        /* 2. Read calibrated sensor snapshots */
        auto& registry = SensorsRegistry::getInstance();
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        registry.getFilteredImuAccel(ax, ay, az);
        registry.getFilteredImuGyro(gx, gy, gz);
        registry.getFilteredImuMag(mx, my, mz);

        /* 3. Gyro unit conversion: °/s → rad/s.
         * gx and gy are negated: LSM9DS1 X/Y gyro axes are inverted relative to
         * the Madgwick convention on this Sense HAT v2 mounting.
         * See Src/Navigator/CoordinateFrames.md for the empirical verification. */
        gx *= -DEG_TO_RAD;
        gy *= -DEG_TO_RAD;
        gz *=  DEG_TO_RAD;

        /* 4. First-call initialisation: seed quaternion from accel + mag instead of
         * identity. This places the filter near the true orientation immediately. */
        if (m_firstCall) {
            initFromSensors(ax, ay, az, mx, my, mz);
            m_firstCall = false;
        }

        /* 5. Linear-acceleration gate.
         * If |accel| deviates from gravity by more than AHRS_MOTION_THRESHOLD the
         * accelerometer is measuring net force, not gravity. Skip both accel and mag
         * corrections this cycle; the ζ bias integrator continues via pure gyro. */
        float accelMag = sqrtf(ax*ax + ay*ay + az*az);
        bool  accelValid = (fabsf(accelMag - AHRS_GRAVITY_MS2) < AHRS_MOTION_THRESHOLD);

        /* 6. Gravity alignment gate.
         * When roll/pitch error is large the 9-DOF gradient couples magnetic and
         * gravity corrections and slows convergence. Run 6-DOF until aligned. */
        float g_est_x = 2.0f*(m_q.x*m_q.z - m_q.w*m_q.y);
        float g_est_y = 2.0f*(m_q.w*m_q.x + m_q.y*m_q.z);
        float g_est_z = 1.0f - 2.0f*m_q.x*m_q.x - 2.0f*m_q.y*m_q.y;
        float ax_n = ax / accelMag, ay_n = ay / accelMag, az_n = az / accelMag;
        float dot  = g_est_x*ax_n + g_est_y*ay_n + g_est_z*az_n;
        if (dot >  1.0f) { dot =  1.0f; }
        if (dot < -1.0f) { dot = -1.0f; }
        float gravityError_deg = RAD_TO_DEG * acosf(dot);
        bool  gravityAligned   = (gravityError_deg < AHRS_GRAVITY_CONVERGENCE_THRESHOLD_DEG);

        /* 7. Magnetometer gates: norm only.
         * The norm gate catches absent/saturated/uncalibrated sensors.
         *
         * NOTE: magUndisturbed() (direction-angle check) is intentionally NOT used
         * here. It rotates the body-frame mag to earth frame using the current
         * quaternion and compares against a stored reference. When the quaternion
         * has accumulated residual gyro-bias drift, the computed earth-frame field
         * appears rotated even though the physical field is unchanged. This causes
         * a false disturbance flag that prematurely disables 9-DOF, which then
         * removes the only Yaw correction path, causing runaway divergence.
         *
         * The gravity alignment gate (gravityAligned, ge < 20°) already prevents
         * 9-DOF from running when the quaternion is too far from reality. That gate
         * is sufficient: it only passes when the quaternion is trustworthy, at which
         * point the direction-angle check would also pass (no contradiction). */
        float magMag    = sqrtf(mx*mx + my*my + mz*mz);
        bool  magNormOk = (magMag >= AHRS_MAG_NORM_MIN && magMag <= AHRS_MAG_NORM_MAX);
        bool  magValid  = accelValid && gravityAligned && magNormOk;

        /* 8. Set effective beta: full value when accel is trustworthy, zero otherwise.
         * With ζ active, the gyro bias continues to be estimated even when beta = 0,
         * so residual bias does not accumulate during dynamic phases. */
        m_filter.setBeta(accelValid ? AHRS_MADGWICK_BETA : 0.0f);

        /* 9. Run filter.
         * Suppress 9-DOF for STARTUP_6DOF_CYCLES: the mag Jacobian s1 cross-term
         * adds to Roll when gz bias has drifted Yaw; 6-DOF lets ζ converge
         * gx/gy first so the coupling is negligible when 9-DOF is enabled. */
        bool useNineDOF = magValid && (m_cycleCount >= STARTUP_6DOF_CYCLES);
        if (useNineDOF && (m_cycleCount == STARTUP_6DOF_CYCLES)) {
            LOG_INFO("[AHRS] Startup 6-DOF phase complete; engaging 9-DOF (Yaw correction active)");
        }
        if (useNineDOF) {
            m_filter.update(m_q, ax, ay, az, gx, gy, gz, mx, my, mz, dt);

            /* Update the magnetic reference with the accepted earth-frame reading.
             * Recompute hx/hy/hz for the EMA (using the updated quaternion). */
            float q0 = m_q.w, q1 = m_q.x, q2 = m_q.y, q3 = m_q.z;
            float rN = 1.0f / magMag;
            float mnx = mx*rN, mny = my*rN, mnz = mz*rN;
            float hx = 2.0f*(mnx*(0.5f - q2*q2 - q3*q3) + mny*(q1*q2 - q0*q3) + mnz*(q1*q3 + q0*q2));
            float hy = 2.0f*(mnx*(q1*q2 + q0*q3) + mny*(0.5f - q1*q1 - q3*q3) + mnz*(q2*q3 - q0*q1));
            float hz = 2.0f*(mnx*(q1*q3 - q0*q2) + mny*(q2*q3 + q0*q1) + mnz*(0.5f - q1*q1 - q2*q2));
            updateMagReference(hx, hy, hz);
        } else {
            m_filter.updateIMU(m_q, ax, ay, az, gx, gy, gz, dt);
        }

        /* 10. Store diagnostics */
        m_lastAccelMag     = accelMag;
        m_lastMagMag       = magMag;
        m_lastMagValid     = useNineDOF;
        m_lastBeta         = accelValid ? AHRS_MADGWICK_BETA : 0.0f;
        m_lastGyroMag      = sqrtf(gx*gx + gy*gy + gz*gz);
        m_lastGravityError = gravityError_deg;

        /* 11. Convert quaternion to Euler angles */
        float roll, pitch, yaw;
        quaternionToEuler(roll, pitch, yaw);

        /* 12. Advance cycle counter and store in registry */
        m_cycleCount++;
        bool ready = (m_cycleCount >= WARMUP_CYCLES);
        registry.updateOrientation(roll, pitch, yaw, ready);
    }

    void AhrsEngine::quaternionToEuler(float& roll_deg,
                                        float& pitch_deg,
                                        float& yaw_deg) const
    {
        /* ZYX Euler extraction from unit quaternion.
         * Coordinate frame: LSM9DS1 (X=forward, Y=left, Z=up).
         *
         * Roll  (X): atan2(2(q0q1 + q2q3), 1 − 2(q1² + q2²))
         * Pitch (Y): asin(clamp(2(q0q2 − q3q1), −1, 1))
         * Yaw   (Z): atan2(2(q0q3 + q1q2), 1 − 2(q2² + q3²)) + declination
         *
         * At Pitch = ±90° the atan2 denominators both → 0 (gimbal lock).
         * Roll and Yaw become co-planar: only their sum/difference is observable.
         * Convention for the degenerate case: set Roll = 0, Yaw absorbs heading.
         *   North pole (+90°): Yaw = +2·atan2(q3, q0)
         *   South pole (−90°): Yaw = −2·atan2(q3, q0)
         * This is a fundamental property of ZYX Euler angles, not a filter bug.
         * Applications that must track through ±90° pitch should use the quaternion
         * directly (available via SensorsRegistry::getOrientation().q). */
        float q0 = m_q.w, q1 = m_q.x, q2 = m_q.y, q3 = m_q.z;

        float sinp = 2.0f*(q0*q2 - q3*q1);
        if (sinp >  1.0f) { sinp =  1.0f; }
        if (sinp < -1.0f) { sinp = -1.0f; }
        pitch_deg = RAD_TO_DEG * asinf(sinp);

        float raw_yaw;
        if (fabsf(sinp) >= 0.9999f) {
            roll_deg = 0.0f;
            raw_yaw  = (sinp > 0.0f) ?  2.0f * RAD_TO_DEG * atan2f(q3, q0)
                                      : -2.0f * RAD_TO_DEG * atan2f(q3, q0);
        } else {
            roll_deg = RAD_TO_DEG * atan2f(2.0f*(q0*q1 + q2*q3),
                                            1.0f - 2.0f*(q1*q1 + q2*q2));
            raw_yaw  = RAD_TO_DEG * atan2f(2.0f*(q0*q3 + q1*q2),
                                             1.0f - 2.0f*(q2*q2 + q3*q3));
        }
        yaw_deg = raw_yaw + m_declination_deg;
    }

    } /* namespace Navigator */
} /* namespace EdgeSense */
