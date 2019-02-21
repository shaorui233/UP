#ifndef WALKING_FORWARD_BODY_ACC_MIN
#define WALKING_FORWARD_BODY_ACC_MIN

#include <Utilities/BSplineBasic.h>
#include <WalkingForward.hpp>
#include <vector>

// First Trial
// Body Trajectory Acceleration is minimized


class BodyAccMin: public WalkingForward{
    public:
        BodyAccMin();
        virtual ~BodyAccMin();

        virtual bool SolveOptimization();
        static double ObjectiveFn(const std::vector<double> &x, 
                std::vector<double> & grad, void *d_);
};

#endif
