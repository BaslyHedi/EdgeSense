/**
 * @file MadgwickFilter.h
 * @author Hedi Basly
 * @brief Madgwick gradient-descent AHRS filter (9-DOF)
 * @date 2026-04-26
 *
 * Reference: S. Madgwick, "An efficient orientation filter for inertial and
 * inertial/magnetic sensor arrays", April 2010.
 * http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 *
 * Required input units:
 *   Accelerometer : m/s²  (any magnitude — normalised internally)
 *   Gyroscope     : rad/s
 *   Magnetometer  : any consistent unit (normalised internally)
 *
 * This class has zero side effects: no registry reads, no logging.
 * It can be unit-tested on any platform without hardware.
 */

#pragma once
#include <EdgeSense/Navigator/OrientationData.h>

namespace EdgeSense {
    namespace Navigator {

    class MadgwickFilter {
    public:

        /* beta: gradient-descent step size.
         * Higher = faster convergence but more noise sensitivity.
         * Typical range [0.01, 0.5]. Madgwick recommends 0.1 as a starting point. */
        explicit MadgwickFilter(float beta = 0.01f);

        /*
         * 9-DOF update (accel + gyro + mag).
         * Modifies q in place. Must be called at a consistent rate; dt is seconds.
         */
        void update(Quaternion& q,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float dt) const;

        /*
         * 6-DOF fallback (accel + gyro only, no mag).
         * Yaw will drift without magnetometer correction. Used when mag is absent.
         */
        void updateIMU(Quaternion& q,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt) const;

        void  setBeta(float beta);
        float getBeta() const;

    private:

        float m_beta;

        /* Fast inverse square root (Quake III variant). Returns 1/sqrt(x). x must be > 0. */
        float invSqrt(float x) const;
    };

    } /* namespace Navigator */
} /* namespace EdgeSense */
