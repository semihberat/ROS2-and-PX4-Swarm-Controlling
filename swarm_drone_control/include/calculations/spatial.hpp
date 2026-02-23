#pragma once
#ifndef SPATIAL_HPP
#define SPATIAL_HPP

#include <cmath>

namespace spatial {

/**
 * @brief Quaternion representation (w, x, y, z)
 * PX4 uses quaternions to avoid gimbal lock
 */
struct Quaternion
{
    double w, x, y, z;
};

/**
 * @brief Euler angles (roll, pitch, yaw) in radians
 * More intuitive for control algorithms
 */
struct EulerAngles
{
    double roll, pitch, yaw; // All in radians
};

/**
 * @brief Normalizes an angle to the [-pi, pi] range.
 * Useful for bearing or heading errors.
 * 
 * @param angle The angle in radians
 * @return The normalized angle in radians
 */
inline double WrapAngleToPi(double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param q Normalized quaternion from PX4
 * @return EulerAngles in radians
 *
 * Uses 3-2-1 (ZYX) Euler sequence.
 * This is the standard aerospace convention.
 */
inline EulerAngles ToEulerAngles(const Quaternion& q)
{
    EulerAngles angles;

    // roll (x-axis rotation) - lateral tilt
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation) - forward/backward tilt
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    // Note: this implementation uses the form sqrt(1+x) and sqrt(1-x) 
    // which handles gimbal lock points mathematically gracefully.
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation) - compass heading
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

/**
 * @brief Overload to easily convert from PX4 msg->q (which is usually a std::array or float[4])
 * @param q Array of 4 floats representing quaternion [w, x, y, z]
 */
template <typename T>
inline EulerAngles ToEulerAngles(const T& q)
{
    // Ensure that T has bracket operator up to index 3
    return ToEulerAngles(Quaternion{
        static_cast<double>(q[0]),
        static_cast<double>(q[1]),
        static_cast<double>(q[2]),
        static_cast<double>(q[3])
    });
}

} // namespace spatial

#endif /* SPATIAL_HPP */
