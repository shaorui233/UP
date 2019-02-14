/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */


#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H


#include "Controllers/StateEstimatorContainer.h"

template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> {
public:
  virtual void run();
};


#endif //PROJECT_POSITIONVELOCITYESTIMATOR_H
