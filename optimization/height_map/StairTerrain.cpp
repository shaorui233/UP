#include "StairTerrain.hpp"

StairTerrain::StairTerrain(int num_stair, double start_x, double start_y, 
        double width, double height, double depth):
    _num_stair(num_stair), _x_start(start_x), _y_start(start_y),
    _w(width), _h(height), _d(depth){

        _stair_tot_depth = _d * num_stair;
        _stair_y_min = _y_start - _w/2.;
        _stair_y_max = _y_start + _w/2.;
    }

double StairTerrain::getHeight(const double& x, const double & y) const{

    if(_stair_y_min < y && y < _stair_y_max){
        if( (_x_start < x) ){
            if(x < (_x_start + _stair_tot_depth) ) {
                // On stair
                for(int i(0); i<_num_stair; ++i){
                    if(x < (_x_start+(i+1)*_d) ){
                        return ((double)(i+1) * _h);
                    }
                }
            }else{
                return ((double)_num_stair * _h);
            }
        }
    }
    return 0.;
}
