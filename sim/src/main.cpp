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
  Simulation sim(RobotType::MINI_CHEETAH, window);

  // add Collision Plane to simulator
  sim.addCollisionPlane(0.8, 0., -0.5);

  // Box 1
  Vec3<double> pos; 
  Mat3<double> ori; 
  Vec3<double> ori_zyx; ori_zyx.setZero();

  pos[0] = 0.0; pos[1] = 0.0; pos[2] = -0.415;

  ori_zyx[0] = 0.3;
  ori_zyx[1] = 0.4;
  EulerZYX_2_SO3(ori_zyx, ori);
  sim.addCollisionBox(0.8, 0., 1.5, 0.7, 0.07, pos, ori);

  // Box 2
  pos[0] = 0.3; pos[1] = 0.05;  pos[2] = -0.30;
  ori_zyx[0] = 0.4; ori_zyx[1] = 0.;  ori_zyx[2] = 0.4;
  EulerZYX_2_SO3(ori_zyx, ori);
  sim.addCollisionBox(0.8, 0., 0.7, 0.7, 0.27, pos, ori);


  // turn on graphics
  window->setAnimating(true);


  // run the simulator with a 10 kHz timestep
  //sim.runAtSpeed(.0001, .0002, .001, 1.);
  
  // run the simulator with a 1 kHz timestep
  sim.runAtSpeed(.001, .002, .001, 0.5);
}

