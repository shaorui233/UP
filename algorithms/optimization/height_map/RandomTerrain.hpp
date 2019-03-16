#ifndef RANDOM_TERRAIN
#define RANDOM_TERRAIN

#include <height_map/HeightMap.hpp>

class RandomTerrain: public HeightMap{
    public:
        RandomTerrain(int num_box, 
                double x_min, double x_max, double y_min, double y_max);
        virtual ~RandomTerrain(){}
        virtual double getHeight(const double & x, const double & y) const ;

    protected:
        int _num_box;
        double _x_min, _x_max; // x min, max
        double _y_min, _y_max; // y min, max

        double _depth_var, _depth_min;
        double _width_var, _width_min;
        double _height_var, _height_min;

        double _default_depth = -0.06;
        std::vector<box_info> _box_info_list;

};

#endif
