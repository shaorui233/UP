#ifndef BOX_TERRAIN
#define BOX_TERRAIN

#include <height_map/HeightMap.hpp>

class StairTerrain: public HeightMap{
    public:
        StairTerrain(int num_stair, double start_x, double start_y, 
                double width, double height, double depth);
        virtual ~StairTerrain(){}
        virtual double getHeight(const double & x, const double & y) const ;

    protected:
        int _num_stair;
        double _x_start;
        double _y_start;
        double _w, _h, _d;

        double _stair_tot_depth;
        double _stair_y_min;
        double _stair_y_max;
};

#endif
