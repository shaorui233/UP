#include <iostream>

#include <RRT_LocYaw.hpp>
#include <Node.hpp>

int main(int argc, char ** argv){
    (void)argc;
    (void)argv;

    LocYaw* ini = new LocYaw(0., 0., 0.);
    LocYaw* fin = new LocYaw(1.6, -1.4, -M_PI*0.95);
    
    RRT* rrt = new RRT_LocYaw(ini, fin);
    rrt->FindPath();

    return 0;
}
