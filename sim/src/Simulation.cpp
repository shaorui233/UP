#include "Simulation.h"


Simulation::Simulation(bool useMiniCheetah, Graphics3D *window) : _tau(12) {
  _quadruped = useMiniCheetah ? buildMiniCheetah<double>() : buildCheetah3<double>();
  _window = window;
  if(_window) {
    _robotID = useMiniCheetah ? window->setupMiniCheetah() : window->setupCheetah3();
  }
  _model = _quadruped.buildModel();
  _simulator = new DynamicsSimulator<double>(_model);

  DVec<double> zero12(12);
  for(u32 i = 0; i < 12; i++) {
    zero12[i] = 0.;
  }

  // set some sane defaults:
  _tau = zero12;
  FBModelState<double> x0;
  x0.bodyOrientation = rotationMatrixToQuaternion(RotMat<double>::Identity());
  x0.bodyPosition = Vec3<double>(0,0,2);
  SVec<double> v0 = SVec<double>::Zero();
  x0.bodyVelocity = v0;
  x0.q = zero12;
  x0.qd = zero12;

  setRobotState(x0);
}

void Simulation::addCollisionPlane(SXform<double>& plane, double mu, double K, double D, bool addToWindow) {
  size_t simulatorID = _simulator->addCollisionPlane(plane, mu, K, D);
  if(addToWindow && _window) {
    Checkerboard checker(20,20,40,40);
    _window->lockGfxMutex();
    size_t graphicsID = _window->_drawList.addCheckerboard(checker);
    _window->_drawList.buildDrawList();
    _window->_drawList.updateCheckerboardFromCollisionPlane(_simulator->getCollisionPlane(simulatorID), graphicsID);
    _window->unlockGfxMutex();
  }
}