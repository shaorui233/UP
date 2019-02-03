/*! @file main.cpp
 *  @brief Main function for simulator
 */


#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"
#include "Utilities/utilities.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/Cheetah3.h"
#include "Collision/CollisionPlane.h"
#include "Graphics3D.h"
#include "DrawList.h"
#include "Dynamics/MiniCheetah.h"
#include "Simulation.h"
#include "SimControlPanel.h"


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

  SimControlPanel panel;
  panel.show();

  // run the Qt program
  a.exec();

  return 0;
}

/*!
 * Run an example simulation
 */
//void simulatorDemo() {
//
//  // simulation of the mini cheetah
//  Simulation sim(RobotType::MINI_CHEETAH, window);
//
//  // Create a V-shaped floor with two planes at +/- 25 degrees
//  //SXform<double> floorLocation1 =
//      //createSXform(coordinateRotation(CoordinateAxis::Y, deg2rad(0.)), Vec3<double>(0,0,-.5));
//  //SXform<double> floorLocation2 = createSXform(coordinateRotation(CoordinateAxis::Y, -deg2rad(20.)), Vec3<double>(0,0,-.5));
//
//  // add planes to simulator
//  //sim.addCollisionPlane(floorLocation1,  0.8, 5e5, 5e3);
//  //sim.addCollisionPlane(floorLocation2,  0.8, 5e5, 5e3);
//
//  // add Collision to simulator
//  sim.addCollisionPlane(0.8, 0., -0.5);
//
//  // Box 1
//  Vec3<double> pos;
//  Mat3<double> ori;
//  Vec3<double> ori_zyx; ori_zyx.setZero();
//
//  pos[0] = 0.0; pos[1] = 0.0; pos[2] = -0.415;
//
//  ori_zyx[0] = 0.3;
//  ori_zyx[1] = 0.4;
//  EulerZYX_2_SO3(ori_zyx, ori);
//  sim.addCollisionBox(0.8, 0., 1.5, 0.7, 0.05, pos, ori);
//
//  // Box 2
//  pos[0] = 0.3; pos[1] = 0.05;  pos[2] = -0.30;
//  ori_zyx[0] = 0.4; ori_zyx[1] = 0.;  ori_zyx[2] = 0.4;
//  EulerZYX_2_SO3(ori_zyx, ori);
//  sim.addCollisionBox(0.8, 0., 0.7, 0.7, 0.05, pos, ori);
//
//
//  // turn on graphics
//  window->setAnimating(true);
//
//
//
//  // run the simulator with a 10 kHz timestep
//  sim.runAtSpeed();
//}

