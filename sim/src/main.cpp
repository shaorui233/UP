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

  // Create a new visualization window
  window = new Graphics3D();
  window->show(); // make window visible
  window->resize(1280, 720);  // set window size

  // run the simulator in a new thread (Qt needs to run a.exec() in the main thread)
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

  // Create a V-shaped floor with two planes at +/- 25 degrees
  SXform<double> floorLocation1 = createSXform(coordinateRotation(CoordinateAxis::Y, deg2rad(25.)), Vec3<double>(0,0,-.5));
  SXform<double> floorLocation2 = createSXform(coordinateRotation(CoordinateAxis::Y, -deg2rad(25.)), Vec3<double>(0,0,-.5));

  // add planes to simulator
  sim.addCollisionPlane(floorLocation1,  0.8, 5e5, 5e3);
  sim.addCollisionPlane(floorLocation2,  0.8, 5e5, 5e3);

  // turn on graphics
  window->setAnimating(true);

  // run the simulator with a 10 kHz timestep
  sim.runAtSpeed(.0001, .0002, .001, 5);
}

