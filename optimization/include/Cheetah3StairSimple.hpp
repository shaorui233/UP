#ifndef CHEETAH3_STAIR_SIMPLE_CLIMBING
#define CHEETAH3_STAIR_SIMPLE_CLIMBING

#include <WalkingPitch.hpp>

class Cheetah3StairSimple: public WalkingPitch{
    public:
        Cheetah3StairSimple();
        virtual ~Cheetah3StairSimple();

        virtual bool SolveOptimization();
        static double ObjectiveFn(const std::vector<double> & x,
                std::vector<double> & grad, void *d_);

        static double ObjectiveFn_LandingMargin(const std::vector<double> & x,
                std::vector<double> & grad, void *d_);

    protected:
        void _ParameterSetting();
        void _SetInitialPitch(std::vector<double> & x);
        int _max_eval;
        int _max_time;
        std::vector<double> _x_initial;

        double _x_start, _y_start;
        double _width, _depth, _height;
        int _num_stair;
};

#endif
