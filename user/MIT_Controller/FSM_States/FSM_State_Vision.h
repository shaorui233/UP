#ifndef FSM_STATE_VISION_H
#define FSM_STATE_VISION_H

#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"
#include <thread>
#include <lcm-cpp.hpp>
#include "heightmap_t.hpp"
#include "rs_pointcloud_t.hpp"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Vision : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Vision(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
  ConvexMPCLocomotion cMPCOld;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

  //void _AddMeshDrawing();
  //void _AddPointsDrawing();

  //void handleVisionLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                       //const heightmap_t* msg);
  //void visionLCMThread();

  //void handlePointsLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                       //const rs_pointcloud_t* msg);
  //void pointsLCMThread();

  //lcm::LCM _visionLCM;
  //std::thread _visionLCMThread;

  //lcm::LCM _pointsLCM;
  //std::thread _pointsLCMThread;

  //bool _b_vision_update = false;

  //DMat<T> _map;
  //Vec3<T> _points[5001];
  //size_t x_size = 100;
  //size_t y_size = 100;
  //size_t num_points = 5001;
  //bool _b_writing = false;
};

#endif  // FSM_STATE_LOCOMOTION_H
