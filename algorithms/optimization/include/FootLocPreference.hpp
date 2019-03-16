#ifndef WALKING_FORWARD_FOOT_LOCATION_PREFERENCE
#define WALKING_FORWARD_FOOT_LOCATION_PREFERENCE

#include <Utilities/BSplineBasic.h>
#include <WalkingForward.hpp>
#include <vector>

// Foot location is prefered to be near to shoulder
// Does not converge well
class FootLocPreference: public WalkingForward{
    public:
        FootLocPreference();
        virtual ~FootLocPreference();

        virtual bool SolveOptimization();
        static double ObjectiveFn(const std::vector<double> &x, 
                std::vector<double> & grad, void *d_);
};

#endif
