#include "Controllers/PositionVelocityEstimator.h"



template<typename T>
void LinearKFPositionVelocityEstimator<T>::run() {
//  T process_noise_pimu = this->_stateEstimatorData.parameters->imu_process_noise_position;
//  T process_noise_vimu = this->_stateEstimatorData.parameters->imu_process_noise_velocity;
//  T process_noise_pfoot = this->_stateEstimatorData.parameters->foot_process_noise_position;
//  T sensor_noise_pimu_rel_foot = this->_stateEstimatorData.parameters->foot_sensor_noise_position;
//  T sensor_noise_vimu_rel_foot = this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
//  T sensor_noise_zfoot = this->_stateEstimatorData.parameters->foot_height_sensor_noise;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;