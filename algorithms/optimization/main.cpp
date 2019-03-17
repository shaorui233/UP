#include <iostream>

#include <Cheetah3StairFootLoc.hpp>

int main(int argc, char ** argv){
    (void)argc;
    (void)argv;

    WalkingFootLoc* walking_opt = new Cheetah3StairFootLoc();

    //for(int i(0); i<1000; ++i) walking_opt->SolveOptimization();

    walking_opt->SolveOptimization();
    return 0;
}
