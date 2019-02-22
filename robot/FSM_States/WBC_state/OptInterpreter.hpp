#ifndef OPTIMIZATION_INTERPRETER
#define OPTIMIZATION_INTERPRETER

#include <cppTypes.h>
#include "include/WalkingForward.hpp"

template <typename T>
class OptInterpreter{
    public:
        static OptInterpreter<T>* getOptInterpreter();
        ~OptInterpreter();


        void updateBodyTarget(const T & t, 
                Vec3<T> & body_target, DVec<T> & body_vel, DVec<T> & body_acc);

        void SetParameter(const std::string & setup_file);
        // 0: fr, hl (6 dim) step location
        // 1: fl, hr 
        // ...
        // 13: fl, hr
        vectorAligned< DVec<T> > _foot_step_list;
        BS_Basic<double, 3, 3, WalkingForward::nMiddle_pt, 2, 2> _body_traj;

        void buildSpline(const std::vector<double> & x);
    private:
        Vec3<T> _start_loc;
        OptInterpreter();
};

#endif
