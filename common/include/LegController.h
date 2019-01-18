/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the body frame,
 *  but is shifted so that 0,0,0 is at the ab/ad pivot.
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include <eigen3/Eigen/Dense>
#include "cppTypes.h"
#include "SpineBoard.h"
#include "Quadruped.h"

template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() {
    zero();
  }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
  Vec3<T> tauEstimate;
};

template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() {
    zero();
  }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
};

template <typename T>
class LegController {
public:
  LegController(Quadruped<T>& quad) : _quadruped(quad)  { }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(const SpiData* spiData);
  void updateCommand(SpiCommand* spiCommand);



  LegControllerCommand<T> commands[4];
  LegControllerData<T>    datas[4];
  Quadruped<T>& _quadruped;
};


template<typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J, Vec3<T>* p, int leg);


#endif //PROJECT_LEGCONTROLLER_H
