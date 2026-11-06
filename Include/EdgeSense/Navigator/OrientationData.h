/**
 * @file OrientationData.h
 * @author Hedi Basly
 * @brief Data structs for AHRS orientation output
 * @date 2026-04-26
 */

#pragma once
#include <cstdint>

namespace EdgeSense {
    namespace Navigator {

    /*
     * Unit quaternion representing 3D orientation.
     * Convention: Hamilton product, w is the scalar component.
     * Identity (no rotation): w=1, x=0, y=0, z=0.
     */
    struct Quaternion {
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    /*
     * Complete orientation output from the AHRS filter.
     *
     * Coordinate frame: LSM9DS1 default (X=forward, Y=left, Z=up).
     * All angles are in degrees, range [-180, +180].
     * The 'valid' flag is false during the filter warm-up period;
     * consumers MUST check valid before using roll/pitch/yaw.
     */
    struct OrientationData {
        Quaternion q;
        float      roll_deg  = 0.0f;
        float      pitch_deg = 0.0f;
        float      yaw_deg   = 0.0f;
        bool       valid     = false;
        uint64_t   timestamp_us = 0;
    };

    } /* namespace Navigator */
} /* namespace EdgeSense */
