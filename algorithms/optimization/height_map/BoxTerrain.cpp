#include "BoxTerrain.hpp"

BoxTerrain::BoxTerrain(double start_x, double start_y, 
        double width, double height, double depth):
    _x_start(start_x), _y_start(start_y),
    _w(width), _h(height), _d(depth){

        _box_y_min = _y_start - _w/2.;
        _box_y_max = _y_start + _w/2.;
    }

double BoxTerrain::getHeight(const double& x, const double & y) const{

    if(_box_y_min < y && y < _box_y_max){
        if( (_x_start < x) && (x < (_x_start + _d)) ){
            return _h;
        }
    }
    return 0.;
}
