/*! @file main.cpp
 *  @brief Main function for simulator
 */


#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "utilities.h"
#include "DynamicsSimulator.h"
#include "Cheetah3.h"
#include "CollisionPlane.h"
#include "Graphics3D.h"
#include "DrawList.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <unistd.h>
#include <thread>
#include <stdio.h>

Graphics3D *window;

/*!
 * An example function showing how a simulation can be set up.
 * @param xpos : starting x position of cheetah
 * @param ypos
 * @param zpos
 */
void runSimulatorTest(double xpos, double ypos, double zpos, int n) {

  usleep((n-12)*500000);
  (void)zpos;


  // build a Quadruped object, which contains various parameters specific to quadrupeds (leg lengths, etc) (todo reorganize so I get this)

  // from this quadruped object, build a rigid body model.
  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();

  // set the gravity to normal gravity
  Vec3<double> g0 = Vec3<double>(0,0,-9.8);
  cheetahModel.setGravity(g0);

  // create a new dynamics simulator based on this model
  DynamicsSimulator<double> sim(cheetahModel);

  // initial conditions
  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, .123) * coordinateRotation(CoordinateAxis::Z, .232) *
                         coordinateRotation(CoordinateAxis::Y, .111);

  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for (size_t i = 0; i < 12; i++) {
    q[i] = 0;
    dq[i] = 0;
    tau[i] = 0;
  }

  // create an initial state for the simulator
  FBModelState<double> x;
  x.bodyOrientation = rotationMatrixToQuaternion(rBody.transpose());
  SVec<double> v0;
  v0 << n, -n, n, .02*n*(n%2?1:-1), .02*n*((n%4<2?1:-1)), .3*n;
  x.bodyVelocity = v0;
  x.bodyPosition = Vec3<double>(xpos,ypos,1);
  x.q = q;
  x.qd = dq;

  // set the initial condition of the simulator
  sim.setState(x);

  // We want a floor for the simulation:
  SXform<double> floorLocation1 = createSXform(coordinateRotation(CoordinateAxis::Y, deg2rad(25.)), Vec3<double>(0,0,-2));
  SXform<double> floorLocation2 = createSXform(coordinateRotation(CoordinateAxis::Y, -deg2rad(25.)), Vec3<double>(0,0,-2));
  size_t collisionPlaneID1 = sim.addCollisionPlane(floorLocation1, 0.8, 5e5, 5e3);
  size_t collisionPlaneID2 = sim.addCollisionPlane(floorLocation2, 0.8, 5e5, 5e3);

  // add a visualization:
  // when adding things to the graphics system, we need to lock the mutex:
  window->lockGfxMutex();

  // add a cheetah to the visualization
  size_t cheetahID = window->setupCheetah3();
  size_t footSphereID = window->_drawList.addDebugSphere(.1f);

  // create a checkerboard for the floor
  Checkerboard c(20,20,40,40);

  // add the checkerboard
  size_t floorID1 = window->_drawList.addCheckerboard(c);
  size_t floorID2 = window->_drawList.addCheckerboard(c);

  // now that we've added everything, we can build the draw list for the graphics program
  window->_drawList.buildDrawList();

  // and also tell the drawlist where the floor should go
  window->_drawList.updateCheckerboardFromCollisionPlane(sim.getCollisionPlane(collisionPlaneID1), floorID1);
  window->_drawList.updateCheckerboardFromCollisionPlane(sim.getCollisionPlane(collisionPlaneID2), floorID2);

  // once this is done, we can go back to drawing frames
  window->unlockGfxMutex();

  uint64_t t = 0;
  float stepsPerSecond;
  qint64 last_frame_ms = 0;

  // simulation loop
  for(;;) {
    t += 1;

    // a "controller"
    for(int i = 0; i < 12; i++)
      tau[i] = - sim.getState().qd[i] - 100*sim.getState().q[i];

    // run the simulator with the controller
    sim.step(0.001, tau);

    // periodically update the simulator graphics
    if(!(t%10)) {
      //window->lockGfxMutex();
      window->_drawList.updateRobotFromModel(sim, cheetahID);
      window->_drawList.updateDebugSphereLocation(sim._pGC[8], footSphereID);
      //window->unlockGfxMutex();
    }
    usleep(1000);
    // periodically print some statistics
    if( !(t%100000) ) {
      qint64 now = QDateTime::currentMSecsSinceEpoch();
      stepsPerSecond = (100000.f / (now - last_frame_ms));
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

  // set up Qt
  QApplication a(argc, argv);

  // set up QSurface, an output for the OpenGL
  QSurfaceFormat gFormat;
  gFormat.setSamples(1);
  gFormat.setDepthBufferSize(24);

  // Create a new visualization window
  window = new Graphics3D();
  window->setFormat(gFormat);
  window->show();
  window->setAnimating(true);
  window->resize(1280, 720);

  // launch a bunch of simulators
  std::vector<std::thread> threadPool;
  int i = 14;
  for(int x = 0; x < 3; x++) {
    for(int y = 0; y < 2; y++) {
      for(int z = 0; z < 1; z++) {
        threadPool.emplace_back(runSimulatorTest, 0,0,0, i++);
      }
    }
  }

  a.exec();

  return 0;
}
