#include <iostream>

//#include <WalkingForward.hpp>
#include <BodyAccMin.hpp>
#include <FootLocPreference.hpp>
#include <Cheetah3StairTop.hpp>
#include <Cheetah3StairForward.hpp>
#include <Cheetah3StairSimple.hpp>

int main(int argc, char ** argv){
    (void)argc;
    (void)argv;

    //WalkingOrientation* walking_opt = new Cheetah3StairTop();
    //WalkingPitch* walking_opt = new Cheetah3StairForward();
    WalkingPitch* walking_opt = new Cheetah3StairSimple();
    //WalkingForward* walking_opt = new BodyAccMin();
    //WalkingForward* walking_opt = new FootLocPreference();

    //for(int i(0); i<1000; ++i) walking_opt->SolveOptimization();

    walking_opt->SolveOptimization();
    return 0;
}
