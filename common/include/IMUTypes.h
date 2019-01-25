/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cppTypes.h"

/*!
 * Cheetah 3's IMU
 */
struct KvhImuData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  // todo add status
};

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat;
  // todo is there status for the vectornav?
};

#endif //PROJECT_IMUTYPES_H
