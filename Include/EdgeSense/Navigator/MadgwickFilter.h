/**
 * @file MadgwickFilter.h
 * @author Hedi Basly
 * @brief Madgwick gradient-descent AHRS filter (9-DOF) with gyro bias estimation.
 * @date 2026-04-26
 *
 * Reference: S. Madgwick, "An efficient orientation filter for inertial and
 * inertial/magnetic sensor arrays", April 2010.
 * http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 *
 * Implements the full Algorithm 3 from the paper, including the ζ (zeta)
 * gyro bias estimator (Eq. 47–49). Both β and ζ are required for correct
 * operation. β alone (as in Algorithm 1) will accumulate residual gyro bias
 * whenever the accel/mag correction is gated off.
 *
 * Required input units:
 *   Accelerometer : m/s²  (any magnitude — normalised internally)
 *   Gyroscope     : rad/s
 *   Magnetometer  : any consistent unit (normalised internally)
 *
 * The filter is stateful: it stores the gyro bias estimate internally.
 * Call resetBias() if the IMU is power-cycled or re-calibrated.
 */

#pragma once
#include <EdgeSense/Navigator/OrientationData.h>

namespace EdgeSense {
    namespace Navigator {

    class MadgwickFilter {
    public:

        /* beta : gradient-descent step size (accel/mag correction rate).
         * zeta : gyro bias estimation rate.
         * Tuning constants are owned by the caller (see AHRS_MADGWICK_BETA /
         * AHRS_MADGWICK_ZETA in AhrsEngine.h). No defaults here to prevent
         * silent divergence between the filter and the engine configuration. */
        MadgwickFilter(float beta, float zeta);

        /* 9-DOF update (accel + gyro + mag). Modifies q and internal bias in place.
         * Bias is applied to gyro before integration (Eq. 47–49). */
        void update(Quaternion& q,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float dt);

        /* 6-DOF fallback (accel + gyro only). Bias is still estimated from the
         * gravity gradient so yaw bias correction continues even without mag. */
        void updateIMU(Quaternion& q,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt);

        void  setBeta(float beta);
        float getBeta() const;
        void  setZeta(float zeta);
        float getZeta() const;

        /* Zero the gyro bias estimate — call after re-calibration. */
        void resetBias();

        /* Read the current bias estimate (rad/s, body frame). */
        void getBias(float& bx, float& by, float& bz) const;

    private:

        float m_beta;
        float m_zeta;

        /* Gyro bias estimate in rad/s — accumulated by the ζ integrator. */
        float m_bx = 0.0f;
        float m_by = 0.0f;
        float m_bz = 0.0f;

        /* Fast inverse square root (Quake III variant). x must be > 0. */
        float invSqrt(float x) const;

        /* Shared bias update: given a normalized gradient [s0..s3] and the current
         * quaternion, integrates the body-frame gyro error into the bias estimate.
         * Uses += so the bias converges toward the true gyro offset (Alg. 3, Eq. 47–49). */
        void updateBias(float q0, float q1, float q2, float q3,
                        float s0, float s1, float s2, float s3,
                        float dt);
    };

    } /* namespace Navigator */
} /* namespace EdgeSense */
