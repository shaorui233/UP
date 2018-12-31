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
#include "Simulation.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <unistd.h>
#include <thread>
#include <stdio.h>

Graphics3D *window;


void simulatorDemo();

/*!
 * Setup QT and run a simulation
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
  std::thread simThread(simulatorDemo);

  // run the Qt program
  a.exec();

  simThread.join();
  return 0;
}

/*!
 * Run an example simulation
 */
void simulatorDemo() {
  // simulation of the mini cheetah
  Simulation sim(true, window);

  // create an initial Floating Base Model State
  FBModelState<double> x;

  // Create a V-shaped floor with two planes at +/- 25 degrees
  SXform<double> floorLocation1 = createSXform(coordinateRotation(CoordinateAxis::Y, deg2rad(25.)), Vec3<double>(0,0,-.5));
  SXform<double> floorLocation2 = createSXform(coordinateRotation(CoordinateAxis::Y, -deg2rad(25.)), Vec3<double>(0,0,-.5));

  // add planes to simulator
  sim.addCollisionPlane(floorLocation1,  0.8, 5e5, 5e3);
  sim.addCollisionPlane(floorLocation2,  0.8, 5e5, 5e3);

  // run simulator
  while(true) {
    sim.step(.00001);
    sim.updateGraphics();
  }
}

