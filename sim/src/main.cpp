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
#include "MiniCheetah.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <unistd.h>
#include <thread>
#include <stdio.h>

Graphics3D *window;


void simulatorDemo(bool useMini);

/*!
 * Hack to launch a simulator graphics window and spawn a separate thread which runs runSimulatorTest
 */
int main(int argc, char *argv[]) {

  // set up Qt
  QApplication a(argc, argv);

  // set up QSurface, an output for the OpenGL-based visualizaer
  QSurfaceFormat gFormat;
  gFormat.setSamples(1);
  gFormat.setDepthBufferSize(24);

  // Create a new visualization window
  window = new Graphics3D();
  window->setFormat(gFormat);
  window->show();
  window->setAnimating(true);
  window->resize(1280, 720);

  // run the simulator in a new thread
  std::thread simMiniThread(simulatorDemo, true);
  std::thread simCheetah3Thread(simulatorDemo, false);

  // run the Qt program
  a.exec();

  // on exit
  simMiniThread.join();
  simCheetah3Thread.join();
  return 0;
}

/*!
 * Demonstration of simulating Cheetah 3 in a "V" shaped terrain.
 */
void simulatorDemo(bool useMini) {
  // create a Cheetah 3 quadruped
  Quadruped<double> cheetahQuadruped;
  if(useMini) {
    cheetahQuadruped = buildMiniCheetah<double>();
  } else {
    cheetahQuadruped = buildCheetah3<double>();
  }


  // build a floating base model
  FloatingBaseModel<double> cheetahModel = cheetahQuadruped.buildModel();

  // create a dynamics simulator for cheetah
  DynamicsSimulator<double> simulator(cheetahModel);

  // initial conditions
  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for(size_t i = 0; i < 12; i++) {
    q[i] = 0;
    dq[i] = 0;
    tau[i] = 0;
  }

  // create an initial Floating Base Model State
  FBModelState<double> x;
  x.bodyOrientation = rotationMatrixToQuaternion(RotMat<double>::Identity());
  x.bodyPosition = Vec3<double>(0,0,1);
  SVec<double> v0;
  v0 << 1,2,3,0,0,0; // some angular velocity to make it interesting
  x.bodyVelocity = v0;
  x.q = q;
  x.qd = dq;

  // set the simulator's initial state
  simulator.setState(x);

  // To create a V-shaped floor we want two 25 degree inclined planes.
  // fist, we create spatial transforms representing their location:
  SXform<double> floorLocation1 = createSXform(coordinateRotation(CoordinateAxis::Y, deg2rad(25.)), Vec3<double>(0,0,-.5));
  SXform<double> floorLocation2 = createSXform(coordinateRotation(CoordinateAxis::Y, -deg2rad(25.)), Vec3<double>(0,0,-.5));

  // next, we add them to the simulator
  size_t collisionPlaneID1 = simulator.addCollisionPlane(floorLocation1, 0.8, 5e5, 5e3);
  size_t collisionPlaneID2 = simulator.addCollisionPlane(floorLocation2, 0.8, 5e5, 5e3);

  // finally, we add everything to the visualization:
  // to make sure we aren't adding stuff as it's drawing a frame, we need to lock the graphics mutex
  window->lockGfxMutex();

  // add the cheetah:
  size_t cheetahID = useMini ? window->setupMiniCheetah() : window->setupCheetah3();

  // add a debugging "balloon"
  size_t debugSphereID = window->_drawList.addDebugSphere(.03f); // radius 3 cm

  // to represent the floor, we use a checkerboard:
  // this must created first
  Checkerboard checkerboard(20,20,40,40); // 20x20 meters, 40x40 checkers

  // then we add this twice to the visualizer (but the visualizer doesn't know where to put these yet)
  size_t floorID1 = window->_drawList.addCheckerboard(checkerboard);
  size_t floorID2 = window->_drawList.addCheckerboard(checkerboard);

  // once we're done adding models, we can tell the graphics window to finalize the model data:
  window->_drawList.buildDrawList();

  // now we can position stuff in our visualizer:
  // we should move the checkerboards to the correct spot
  window->_drawList.updateCheckerboardFromCollisionPlane(simulator.getCollisionPlane(collisionPlaneID1), floorID1);
  window->_drawList.updateCheckerboardFromCollisionPlane(simulator.getCollisionPlane(collisionPlaneID2), floorID2);

  // now we can unlock the mutex:
  window->unlockGfxMutex();


  // finally, we can run the simulator:
  uint64_t stepCount = 0;
  for(;;) {
    // a "controller" to make the legs thrash around
    for(int i = 0; i < 12; i++) {
      //tau[i] = 1 - simulator.getState().qd[i];
      tau[i] = -simulator.getState().qd[i] - simulator.getState().q[i]*10;
    }

    // step the simulator forward 1 ms
    simulator.step(.001, tau);

    // every 16 steps, update the graphics:
    if(!(stepCount%16)) {
      // the floating base models can be grabbed automatically from the simulator
      window->_drawList.updateRobotFromModel(simulator, cheetahID);
      // also we can put a debugging sphere at ground contact point 9 (foot 1)
      window->_drawList.updateDebugSphereLocation(simulator._pGC[9], debugSphereID);
    }

    // sleep for 1 ms to make the simulator run near the correct speed.
    usleep(1000);

    stepCount++;
  }
}
