#ifndef BOX_TERRAIN
#define BOX_TERRAIN

#include <height_map/HeightMap.hpp>

class BoxTerrain: public HeightMap{
    public:
        BoxTerrain(double start_x, double start_y, 
                double width, double height, double depth);
        virtual ~BoxTerrain(){}
        virtual double getHeight(const double & x, const double & y) const ;

    protected:
        double _x_start;
        double _y_start;
        double _w, _h, _d;

        double _box_y_min;
        double _box_y_max;
};

#endif
