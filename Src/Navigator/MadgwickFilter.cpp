/**
 * @file MadgwickFilter.cpp
 * @author Hedi Basly
 * @brief Madgwick 9-DOF gradient-descent orientation filter with gyro bias estimation.
 * @date 2026-04-26
 *
 * Equations follow the Madgwick 2010 paper (Algorithm 3).
 * Variable names preserve the paper's notation where practical.
 *
 * Key difference from the naive implementation (Algorithm 1):
 *   The ζ gyro bias estimator (Eq. 47–49) accumulates the body-frame angular error
 *   derived from the gradient into a running bias correction. This correction is
 *   subtracted from the raw gyro reading before integration so that residual bias
 *   does not accumulate when β is reduced or gated.
 *
 *   Bias rate (vector part of 2·q*⊗ŝ, per Madgwick 2010 Algorithm 3):
 *     bDot_x = +2ζ · (q0·s1 − q1·s0 − q2·s3 + q3·s2)
 *     bDot_y = +2ζ · (q0·s2 + q1·s3 − q2·s0 − q3·s1)
 *     bDot_z = +2ζ · (q0·s3 − q1·s2 + q2·s1 − q3·s0)
 */

#include <EdgeSense/Navigator/MadgwickFilter.h>
#include <cmath>
#include <cstring>

namespace EdgeSense {
    namespace Navigator {

    MadgwickFilter::MadgwickFilter(float beta, float zeta)
        : m_beta(beta), m_zeta(zeta) {}

    void  MadgwickFilter::setBeta(float beta) { m_beta = beta; }
    float MadgwickFilter::getBeta()  const    { return m_beta; }
    void  MadgwickFilter::setZeta(float zeta) { m_zeta = zeta; }
    float MadgwickFilter::getZeta()  const    { return m_zeta; }
    void  MadgwickFilter::resetBias()         { m_bx = 0.0f; m_by = 0.0f; m_bz = 0.0f; }
    void  MadgwickFilter::getBias(float& bx, float& by, float& bz) const
    {
        bx = m_bx; by = m_by; bz = m_bz;
    }

    float MadgwickFilter::invSqrt(float x) const {
        float halfx = 0.5f * x;
        float y = x;
        int32_t i;
        std::memcpy(&i, &y, sizeof(float));
        i = 0x5f3759df - (i >> 1);
        std::memcpy(&y, &i, sizeof(float));
        y = y * (1.5f - (halfx * y * y));
        return y;
    }

    void MadgwickFilter::updateBias(float q0, float q1, float q2, float q3,
                                     float s0, float s1, float s2, float s3,
                                     float dt)
    {
        /* Eq. 47–49: body-frame gyro error = vector part of 2·q*⊗ŝ.
         * Bias converges toward the true gyro offset: b_{k+1} = b_k + 2ζ·e_w·dt
         * so that (gyro_raw - b) → 0 over time.
         * Sign must be += (additive) per Madgwick 2010 Algorithm 3 and the x-io
         * reference implementation. A -= here inverts the integrator and amplifies
         * residual bias instead of cancelling it. */
        float twoZeta = 2.0f * m_zeta;
        m_bx += twoZeta * (q0*s1 - q1*s0 - q2*s3 + q3*s2) * dt;
        m_by += twoZeta * (q0*s2 + q1*s3 - q2*s0 - q3*s1) * dt;
        m_bz += twoZeta * (q0*s3 - q1*s2 + q2*s1 - q3*s0) * dt;
    }

    void MadgwickFilter::update(Quaternion& q,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz,
                                float dt)
    {
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        bool  hasGradient = false;

        float accelNorm = ax*ax + ay*ay + az*az;
        if (accelNorm > 0.0f) {
            float recipNorm = invSqrt(accelNorm);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            float magNorm = mx*mx + my*my + mz*mz;
            if (magNorm > 0.0f) {

                recipNorm = invSqrt(magNorm);
                mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

                /* Reference direction of Earth's magnetic field (Eq. 45–46).
                 * Project measured field to Earth frame, then flatten to horizontal (bx)
                 * and vertical (bz), discarding hy to make yaw observable. */
                float hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
                float hy = 2.0f*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
                float bx = sqrtf(hx*hx + hy*hy);
                float bz = 2.0f*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

                /* 9-DOF gradient (Eq. 25 + Eq. 44): J_g^T·f_g + J_b^T·f_b */
                s0 = -2.0f*q2*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q1*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q0*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   + -2.0f*bz*q2*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   + (-2.0f*bx*q3 + 2.0f*bz*q1)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +  2.0f*bx*q2*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s1 =  2.0f*q3*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q0*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q1*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   +  2.0f*bz*q3*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   + (2.0f*bx*q2 + 2.0f*bz*q0)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   + (2.0f*bx*q3 - 4.0f*bz*q1)*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s2 = -2.0f*q0*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q3*(2.0f*(q0*q1 + q2*q3) - ay)
                   + -4.0f*q2*(1.0f - 2.0f*(q1*q1 + q2*q2) - az)
                   + (-4.0f*bx*q2 - 2.0f*bz*q0)*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   + (2.0f*bx*q1 + 2.0f*bz*q3)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   + (2.0f*bx*q0 - 4.0f*bz*q2)*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

                s3 =  2.0f*q1*(2.0f*(q1*q3 - q0*q2) - ax)
                   +  2.0f*q2*(2.0f*(q0*q1 + q2*q3) - ay)
                   + (-4.0f*bx*q3 + 2.0f*bz*q1)*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx)
                   + (-2.0f*bx*q0 + 2.0f*bz*q2)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my)
                   +  2.0f*bx*q1*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2) - mz);

            } else {
                /* 6-DOF gravity-only gradient (Eq. 25) */
                s0 = -2.0f*q2*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q1*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q0*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
                s1 =  2.0f*q3*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q0*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q1*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
                s2 = -2.0f*q0*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q3*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q2*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
                s3 =  2.0f*q1*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q2*(2.0f*(q0*q1 + q2*q3) - ay);
            }

            recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

            /* Update gyro bias estimate from gradient before applying feedback */
            updateBias(q0, q1, q2, q3, s0, s1, s2, s3, dt);
            hasGradient = true;
        }

        /* Apply accumulated bias correction to raw gyro (Eq. 49) */
        gx -= m_bx; gy -= m_by; gz -= m_bz;

        /* Rate of change from corrected gyro (Eq. 12) */
        float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

        /* Apply gradient feedback (Eq. 33) */
        if (hasGradient) {
            qDot0 -= m_beta * s0;
            qDot1 -= m_beta * s1;
            qDot2 -= m_beta * s2;
            qDot3 -= m_beta * s3;
        }

        /* Integrate (Eq. 13) */
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        float recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q.w = q0 * recipNorm;
        q.x = q1 * recipNorm;
        q.y = q2 * recipNorm;
        q.z = q3 * recipNorm;
    }

    void MadgwickFilter::updateIMU(Quaternion& q,
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float dt)
    {
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        bool  hasGradient = false;

        float accelNorm = ax*ax + ay*ay + az*az;
        if (accelNorm > 0.0f) {
            float recipNorm = invSqrt(accelNorm);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            s0 = -2.0f*q2*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q1*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q0*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s1 =  2.0f*q3*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q0*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q1*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s2 = -2.0f*q0*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q3*(2.0f*(q0*q1 + q2*q3) - ay) + -4.0f*q2*(1.0f - 2.0f*(q1*q1 + q2*q2) - az);
            s3 =  2.0f*q1*(2.0f*(q1*q3 - q0*q2) - ax) + 2.0f*q2*(2.0f*(q0*q1 + q2*q3) - ay);

            recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

            updateBias(q0, q1, q2, q3, s0, s1, s2, s3, dt);
            hasGradient = true;
        }

        gx -= m_bx; gy -= m_by; gz -= m_bz;

        float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

        if (hasGradient) {
            qDot0 -= m_beta * s0;
            qDot1 -= m_beta * s1;
            qDot2 -= m_beta * s2;
            qDot3 -= m_beta * s3;
        }

        q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
        float recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q.w = q0*recipNorm; q.x = q1*recipNorm; q.y = q2*recipNorm; q.z = q3*recipNorm;
    }

    } /* namespace Navigator */
} /* namespace EdgeSense */
