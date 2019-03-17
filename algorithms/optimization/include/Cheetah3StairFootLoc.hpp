#ifndef CHEETAH3_STAIR_FOOT_LOCATION
#define CHEETAH3_STAIR_FOOT_LOCATION

#include <WalkingFootLoc.hpp>

class Cheetah3StairFootLoc: public WalkingFootLoc{
    public:
        Cheetah3StairFootLoc();
        virtual ~Cheetah3StairFootLoc();

        virtual bool SolveOptimization();
        static double ObjectiveFn(const std::vector<double> & x,
                std::vector<double> & grad, void *d_);

        void _update_full_variable(const std::vector<double> & x);

        static void StickToInitialConstraint(
                unsigned m, double * result, unsigned n, const double *x, 
                double * grad, void*data);
        
        std::vector<double> _x_initial;

    protected:
        void _ParameterSetting();
        void _SetInitialPitch(std::vector<double> & x);
        int _max_eval;
        int _max_time;
        std::vector<double> _full_x;

        double _x_start, _y_start;
        double _width, _depth, _height;
        int _num_stair;
};

#endif
