/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include <stdio.h>
#include <QApplication>


#include <Graphics3D.h>
#include <DrawList.h>
#include <QSurfaceFormat>
#include <FloatingBaseModel.h>
#include <Quadruped.h>
#include <utilities.h>
#include "DynamicsSimulator.h"
#include <Cheetah3.h>
#include <unistd.h>

#include <thread>

Graphics3D *window;

void runSimulatorTest() {

  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();
  Vec3<double> g0 = Vec3<double>(0,0,0);
  cheetahModel.setGravity(g0);
  DynamicsSimulator<double> sim(cheetahModel);
  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, 0*.123) * coordinateRotation(CoordinateAxis::Z, 0*.232) *
                         coordinateRotation(CoordinateAxis::Y, 0*.111);
  FBModelState<double> x;
  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for (size_t i = 0; i < 12; i++) {
    q[i] = 0;
    dq[i] = 0;
    tau[i] = 0;
  }

  // set state
  x.bodyOrientation = rotationMatrixToQuaternion(rBody.transpose());
  x.bodyVelocity = SVec<double>::Zero();
  x.bodyPosition = Vec3<double>(0,0,0);
  x.q = q;
  x.qd = dq;

  window->lockGfxMutex();
  // add cheetah to simulator
  window->setupCheetah3();
  window->unlockGfxMutex();

  // step simulator to get forward kinematics to run
  sim.setState(x);

  uint64_t t = 0;
  uint64_t stepsPerSecond;
  qint64 last_frame_ms = 0;
  for(;;) {
    int abads[4] = {0,3,6,9};
    for(int i = 0; i < 4; i++)
      tau[abads[i]] = .3 - sim.getState().qd[abads[i]];
    t += 1;
    sim.step(0.00007, tau);
    if(!(t%160)) {
      //window->lockGfxMutex();
      window->_drawList.updateRobotFromModel(sim, 0);
      //window->unlockGfxMutex();
    }
    if( !(t%10000) ) {
      qint64 now = QDateTime::currentMSecsSinceEpoch();
      //printf("%f\n", t * 0.00007);
      stepsPerSecond = (10000.f / (now - last_frame_ms));
      std::cout << "Timesteps per ms: " << stepsPerSecond << "\n";
      last_frame_ms = now;
    }
    if(t > 1e10) break;
  }

}

/*!
 * Hack to launch a simulator graphics window and spawn a separate thread which runs runSimulatorTest
 */
int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  QSurfaceFormat gFormat;
  gFormat.setSamples(1);
  gFormat.setDepthBufferSize(24);
  window = new Graphics3D();

  window->setFormat(gFormat);
  window->show();
  window->setAnimating(true);
  window->resize(1280, 720);

  std::thread t(runSimulatorTest);
  a.exec();

  t.join();
  return 1;
}
