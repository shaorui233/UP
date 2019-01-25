/*! @file ImuSimulator.h
 *  @brief Simulated IMU
 */
#include "ImuSimulator.h"
#include "orientation_tools.h"
#include "spatial.h"
#include "utilities.h"


template <typename T>
void ImuSimulator<T>::computeAcceleration(const FBModelState<T> &robotState,
                                          const FBModelStateDerivative<T> &robotStateD,
                                          Vec3<float> &acc,
                                          std::uniform_real_distribution<float> &dist,
                                          const RotMat<float>& R_body) {

  // accelerometer noise
  fillEigenWithRandom(acc, _mt, dist);

  // gravity (should be positive when robot is upright)
  acc += (R_body * Vec3<float>(0,0,9.81));

  // acceleration (todo check with pat that this should work)
  acc += spatial::sAccToClassicalAcc(robotStateD.dBodyVelocity, robotState.bodyVelocity).template cast<float>();
}

template <typename T>
void ImuSimulator<T>::updateKVH(const FBModelState<T> &robotState, const FBModelStateDerivative<T>& robotStateD, KvhImuData *data) {

  // body orientation
  RotMat<float> R_body = quaternionToRotationMatrix(robotState.bodyOrientation.template cast<float>());

  // acceleration
  computeAcceleration(robotState, robotStateD, data->accelerometer, _kvhAccelerometerDistribution, R_body);

  // gyro
  fillEigenWithRandom(data->gyro, _mt, _kvhGyroDistribution);
  data->gyro += robotState.bodyVelocity.template head<3>().template cast<float>();
}

template <typename T>
void ImuSimulator<T>::updateVectornav(const FBModelState<T> &robotState, const FBModelStateDerivative<T>& robotStateD, VectorNavData *data) {
  // body orientation
  RotMat<float> R_body = quaternionToRotationMatrix(robotState.bodyOrientation.template cast<float>());

  // acceleration
  computeAcceleration(robotState, robotStateD, data->accelerometer, _vectornavAccelerometerDistribution, R_body);

  // gyro
  fillEigenWithRandom(data->gyro, _mt, _vectornavGyroDistribution);
  data->gyro += robotState.bodyVelocity.template head<3>().template cast<float>();

  // quaternion
  if(vectorNavNoise) {
    Vec3<float> omegaNoise;
    fillEigenWithRandom(omegaNoise, _mt, _vectornavQuatDistribution);
    Quat<float> floatQuat = robotState.bodyOrientation.template cast<float>();
    data->quat = integrateQuat(floatQuat, omegaNoise, 1.0f);
  } else {
    data->quat = robotState.bodyOrientation.template cast<float>();
  }
}


template class ImuSimulator<float>;
template class ImuSimulator<double>;