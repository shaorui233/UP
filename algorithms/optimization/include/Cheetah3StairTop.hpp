#ifndef CHEETAH3_STAIR_CLIMBING
#define CHEETAH3_STAIR_CLIMBING

#include <WalkingOrientation.hpp>

class Cheetah3StairTop: public WalkingOrientation{
    public:
        Cheetah3StairTop();
        virtual ~Cheetah3StairTop();

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
