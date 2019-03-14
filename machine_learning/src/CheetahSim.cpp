#include <iostream>
#include <stdio.h>
#include <DynSimulation.hpp>
#include <QApplication>

SimGraphics3D* window;
DynSimulation* dyn_sim;

void RunSimulator(){ 
    dyn_sim = new DynSimulation(window); 
    window->setAnimating(true);
    dyn_sim->run();
}

int main(int argc, char** argv)
{
    // Simulation
    QApplication a(argc, argv);
    window = new SimGraphics3D();
    window->show();
    window->resize(1280, 760);
    std::thread testThread(RunSimulator);
    a.exec();
    testThread.join();

    return 0;
}

