#ifndef SEPARATE_TERRAIN
#define SEPARATE_TERRAIN

#include <height_map/HeightMap.hpp>

class SeparateTerrain: public HeightMap{

    public:
        SeparateTerrain():HeightMap(){}
        virtual ~SeparateTerrain(){}
        virtual double getHeight(const double& x, const double & y) const {
            (void)y;
            if(x<first_sec_end){ return 0.; }
            else if(x<second_sec_start){ return -0.5; }
            else if(x<second_sec_end){ return 0.0; }
            else if(x<third_sec_start){ return -0.5; }
            else { return 0.0; }
        }
        double first_sec_end = 0.47;
        double second_sec_start = 0.53;
        double second_sec_end = 1.08;
        double third_sec_start = 1.15;
};

#endif

