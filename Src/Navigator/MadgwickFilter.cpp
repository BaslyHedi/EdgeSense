/**
 * @file MadgwickFilter.cpp
 * @author Hedi Basly
 * @brief Madgwick 9-DOF gradient-descent orientation filter
 * @date 2026-04-26
 *
 * Equations follow the Madgwick 2010 paper exactly.
 * Variable names preserve the paper's notation (q0..q3, s0..s3, bx, bz).
 */

#include <EdgeSense/Navigator/MadgwickFilter.h>
#include <cmath>
#include <cstring>

namespace EdgeSense {
    namespace Navigator {

    MadgwickFilter::MadgwickFilter(float beta)
        : m_beta(beta) {}

    void MadgwickFilter::setBeta(float beta) { m_beta = beta; }
    float MadgwickFilter::getBeta() const    { return m_beta; }

    float MadgwickFilter::invSqrt(float x) const {
        /* Quake III fast inverse square root.
         * memcpy used for type-safe bit reinterpretation (defined in C++17). */
        float halfx = 0.5f * x;
        float y = x;
        int32_t i;
        std::memcpy(&i, &y, sizeof(float));
        i = 0x5f3759df - (i >> 1);
        std::memcpy(&y, &i, sizeof(float));
        y = y * (1.5f - (halfx * y * y)); /* one Newton-Raphson iteration */
        return y;
    }

    void MadgwickFilter::update(Quaternion& q,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz,
                                float dt) const
    {
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot0, qDot1, qDot2, qDot3;

        /* Rate of change of quaternion from gyroscope — Eq. 12:
         * qDot = 0.5 * q ⊗ [0, gx, gy, gz] */
        qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

        float accelNorm = ax*ax + ay*ay + az*az;
        if (accelNorm > 0.0f) {

            /* Normalise accelerometer */
            recipNorm = invSqrt(accelNorm);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            float magNorm = mx*mx + my*my + mz*mz;
            if (magNorm > 0.0f) {

                /* Normalise magnetometer */
                recipNorm = invSqrt(magNorm);
                mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

                /* Reference direction of Earth's magnetic field — Eq. 45-46.
                 * Rotate mag into Earth frame via current q, retain only bx (horizontal)
                 * and bz (vertical) to make yaw-observable while ignoring dip angle. */
                float hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
                float hy = 2.0f*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
                float bx = sqrtf(hx*hx + hy*hy);
                float bz = 2.0f*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

                /* Gradient descent corrective step — Eq. 25, 34, 44:
                 * ∇f = J_g^T * f_g + J_b^T * f_b */
                s0 = -2.0f*q2*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q1*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q0*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   +  -2.0f*bz*q2*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   +  (-2.0f*bx*q3 + 2.0f*bz*q1)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +   2.0f*bx*q2*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s1 =  2.0f*q3*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q0*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q1*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   +  2.0f*bz*q3*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   +  (2.0f*bx*q2 + 2.0f*bz*q0)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +  (2.0f*bx*q3 - 4.0f*bz*q1)*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s2 = -2.0f*q0*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q3*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q2*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   +  (-4.0f*bx*q2 - 2.0f*bz*q0)*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   +  (2.0f*bx*q1 + 2.0f*bz*q3)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +  (2.0f*bx*q0 - 4.0f*bz*q2)*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s3 =  2.0f*q1*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q2*(2.0f*(q0*q1 + q2*q3) - ay)
                   +  (-4.0f*bx*q3 + 2.0f*bz*q1)*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   +  (-2.0f*bx*q0 + 2.0f*bz*q2)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +   2.0f*bx*q1*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                /* Normalise gradient step magnitude */
                recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
                s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

                /* Apply feedback: qDot -= beta * gradient — Eq. 33 */
                qDot0 -= m_beta * s0;
                qDot1 -= m_beta * s1;
                qDot2 -= m_beta * s2;
                qDot3 -= m_beta * s3;
            }
            /* If mag is invalid: only gyro integration this cycle (no yaw correction) */
        }

        /* Integrate: q(t+dt) = q(t) + qDot*dt — Eq. 13 */
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        /* Re-normalise quaternion (unit constraint) */
        recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q.w = q0 * recipNorm;
        q.x = q1 * recipNorm;
        q.y = q2 * recipNorm;
        q.z = q3 * recipNorm;
    }

    void MadgwickFilter::updateIMU(Quaternion& q,
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float dt) const
    {
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        float recipNorm;
        float s0, s1, s2, s3;

        float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

        float accelNorm = ax*ax + ay*ay + az*az;
        if (accelNorm > 0.0f) {
            recipNorm = invSqrt(accelNorm);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            /* Gravity objective function — 6-DOF only (Eq. 25) */
            s0 = -2.0f*q2*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q1*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q0*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s1 =  2.0f*q3*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q0*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q1*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s2 = -2.0f*q0*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q3*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q2*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s3 =  2.0f*q1*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q2*(2.0f*(q0*q1 + q2*q3) - ay);

            recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

            qDot0 -= m_beta * s0;
            qDot1 -= m_beta * s1;
            qDot2 -= m_beta * s2;
            qDot3 -= m_beta * s3;
        }

        q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
        recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q.w = q0*recipNorm; q.x = q1*recipNorm; q.y = q2*recipNorm; q.z = q3*recipNorm;
    }

    } /* namespace Navigator */
} /* namespace EdgeSense */
