#ifndef OPTIMIZATION_INTERPRETER
#define OPTIMIZATION_INTERPRETER

#include <cppTypes.h>

#include "include/WalkingForward.hpp"
#include "include/WalkingOrientation.hpp"
#include "include/WalkingPitch.hpp"

//#define OptCase WalkingOrientation
#define OptCase WalkingPitch
#define OriSplineDim 1

template <typename T>
class OptInterpreter{
    public:
        static OptInterpreter<T>* getOptInterpreter();
        ~OptInterpreter();


        void updateBodyTarget(const T & t, 
                Vec3<T> & body_target, DVec<T> & body_vel, DVec<T> & body_acc);

        void updateBodyOriTarget(const T & t, Vec3<T> & ori_target);

        void SetParameter(const std::string & setup_file);
        // 0: fr, hl (6 dim) step location
        // 1: fl, hr 
        // ...
        // 13: fl, hr
        vectorAligned< DVec<T> > _foot_step_list;
        BS_Basic<double, 3, 3, OptCase::nMiddle_pt, 2, 2> _body_traj;
        BS_Basic<double, OptCase::dimOri, 3, OptCase::nMiddle_pt, 2, 2> _body_ori_traj;

        void buildSpline(const std::vector<double> & x);
        void buildOrientationSpline(const std::vector<double> & x); 
    private:
        Vec3<T> _start_loc;
        OptInterpreter();
};

#endif
