#include "RandomTerrain.hpp"
#include <cstdlib>

RandomTerrain::RandomTerrain(int num_box, 
        double x_min, double x_max, double y_min, double y_max): 
    HeightMap(),
    _num_box(num_box),
    _x_min(x_min), _x_max(x_max),
    _y_min(y_min), _y_max(y_max)
{
    box_info box;

    _depth_var = 0.35; _depth_min = 0.1;
    _width_var = 0.35; _width_min = 0.1;
    _height_var = 0.07; _height_min = -0.035;

    // larger area cover
    double cover_area_extension(0.2);
    double x_gap(_x_max - _x_min + cover_area_extension);
    double y_gap(_y_max - _y_min + cover_area_extension);

    srand (time(NULL));
    for(int i(0); i<_num_box; ++i){
        box.x_right_bottom = x_gap * (double)rand()/RAND_MAX + 
            (_x_min - cover_area_extension);
        box.y_right_bottom = y_gap * (double)rand()/RAND_MAX + (_y_min - cover_area_extension);

        box.depth = _depth_var * (double)rand()/RAND_MAX + _depth_min;
        box.width = _width_var * (double)rand()/RAND_MAX + _width_min;
        
        if(box.x_right_bottom + box.depth > _x_max){
            box.depth = _x_max - box.x_right_bottom;
        }

        if(box.y_right_bottom + box.width > _y_max){
            box.width = _y_max - box.y_right_bottom;
        }
        box.height = _height_var * (double)rand()/RAND_MAX + _height_min;

        _box_info_list.push_back(box);
    }
}

double RandomTerrain::getHeight(const double & x, const double & y) const {

    if(x<_x_min){ return 0.; }
    if(x>_x_max){ return 0.; }
    
    double ret = _default_depth;

    std::vector<box_info>::const_iterator iter = _box_info_list.begin();
    static int count(0);
    ++count;
    for(; iter<_box_info_list.end(); ++iter){
        if(count == 1)           iter->show_info();
        if( (iter->x_right_bottom < x) && (x<(iter->x_right_bottom + iter->depth)) ){
            if((iter->y_right_bottom<y) && (y<(iter->y_right_bottom + iter->width)) ){
                if(iter->height > ret){
                    ret = iter->height;
                    //printf("curr loc: %f, %f\n", x, y);
                    //iter->show_info();
                }
            }
        }
    }

    return ret;
}
