/*! @file ImuSimulator.h
 *  @brief Simulated IMU
 */
#include "SimUtilities/ImuSimulator.h"
#include "Math/orientation_tools.h"
#include "Dynamics/spatial.h"
#include "Utilities/utilities.h"


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
  acc += spatial::spatialToLinearAcceleration(robotStateD.dBodyVelocity, robotState.bodyVelocity).template cast<float>();
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
  if(_vectorNavOrientationNoise) {
    Vec3<float> omegaNoise;
    fillEigenWithRandom(omegaNoise, _mt, _vectornavQuatDistribution);
    Quat<float> floatQuat = robotState.bodyOrientation.template cast<float>();
    data->quat = integrateQuat(floatQuat, omegaNoise, 1.0f);
  } else {
    data->quat = robotState.bodyOrientation.template cast<float>();
  }
}

template <typename T>
void ImuSimulator<T>::updateCheaterState(const FBModelState<T> &robotState, const FBModelStateDerivative<T>& robotStateD, CheaterState<T> &state) {
  RotMat<T> R_body = quaternionToRotationMatrix(robotState.bodyOrientation);
  state.acceleration = (R_body * Vec3<T>(0,0,9.81)) + spatial::spatialToLinearAcceleration(robotStateD.dBodyVelocity, robotState.bodyVelocity);
  state.orientation = robotState.bodyOrientation;
  state.position = robotState.bodyPosition;
  state.omegaBody = robotState.bodyVelocity.template head<3>();
  state.vBody = robotState.bodyVelocity.template tail<3>();
}


template class ImuSimulator<float>;
template class ImuSimulator<double>;