#ifndef CHEETAH3_STAIR_FORWARD_CLIMBING
#define CHEETAH3_STAIR_FORWARD_CLIMBING

#include <WalkingPitch.hpp>

class Cheetah3StairForward: public WalkingPitch{
    public:
        Cheetah3StairForward();
        virtual ~Cheetah3StairForward();

        virtual bool SolveOptimization();
        static double ObjectiveFn(const std::vector<double> & x,
                std::vector<double> & grad, void *d_);

    protected:
        void _ParameterSetting();
        int _max_eval;
        int _max_time;

        double _x_start, _y_start;
        double _width, _depth, _height;
        int _num_stair;
};

#endif
