#include <iostream>

//#include <WalkingForward.hpp>
#include <BodyAccMin.hpp>
#include <FootLocPreference.hpp>

int main(int argc, char ** argv){
    (void)argc;
    (void)argv;

    WalkingForward* walking_opt = new BodyAccMin();
    //WalkingForward* walking_opt = new FootLocPreference();

    for(int i(0); i<1000; ++i) walking_opt->SolveOptimization();

    return 0;
}
