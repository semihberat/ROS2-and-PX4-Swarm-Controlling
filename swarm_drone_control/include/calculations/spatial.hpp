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
inline double WrapAngleToPi(double angle);

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param q Normalized quaternion from PX4
 * @return EulerAngles in radians
 *
 * Uses 3-2-1 (ZYX) Euler sequence.
 * This is the standard aerospace convention.
 */
inline EulerAngles ToEulerAngles(const Quaternion& q);

/**
 * @brief Overload to easily convert from PX4 msg->q (which is usually a std::array or float[4])
 * @param q Array of 4 floats representing quaternion [w, x, y, z]
 */
template <typename T>
inline EulerAngles ToEulerAngles(const T& q);

} // namespace spatial

// Include implementation
#include "spatial_impl.hpp"

#endif /* SPATIAL_HPP */
