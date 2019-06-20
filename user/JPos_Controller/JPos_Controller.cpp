#include "JPos_Controller.hpp"

void JPos_Controller::runController(){

  Mat3<float> kpMat;
  Mat3<float> kdMat;
  kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
  kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;

  for(int leg(0); leg<4; ++leg){
    for(int jidx(0); jidx<3; ++jidx){
      _legController->commands[leg].qDes[jidx] = 0.;
      _legController->commands[leg].qdDes[jidx] = 0.;
    }
    _legController->commands[leg].kpJoint = kpMat;
    _legController->commands[leg].kdJoint = kdMat;
  }
}
