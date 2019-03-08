#ifndef HEIGHT_MAP_H
#define HEIGHT_MAP_H

#include <utilities/save_file.hpp>

class box_info{
    public:
        box_info(){}
        ~box_info(){}
        double x_right_bottom;
        double y_right_bottom;
        double depth; // x
        double width; // y
        double height; // z
        void show_info() const{
            printf("x0, y0, x1, y1, height: (%f, %f, %f, %f, %f)\n", 
                    x_right_bottom,
                    y_right_bottom,
                    x_right_bottom+ depth,
                    y_right_bottom + width,
                    height);
        }
};

class HeightMap{
    public:
        HeightMap(){}
        virtual ~HeightMap(){}
        virtual double getHeight(const double & x, const double & y) const = 0;

        //constexpr static size_t y_grid = 120; //0.6m / 0.005 m  = 300 (grid)
        //constexpr static size_t x_grid = 320; //1.6m / 0.005 m  = 800 (grid)

        //constexpr static double dx = 1.6/x_grid;
        //constexpr static double dy = 0.6/y_grid;

        constexpr static double x_offset = 0.;
        constexpr static double y_offset = -2.2;
        constexpr static size_t y_grid = 300; //3.0m / 0.03 m  = 100 (grid)
        constexpr static size_t x_grid = 300; //3.0m / 0.03 m  = 100 (grid)

        constexpr static double dx = 0.01;
        constexpr static double dy = 0.01;


        void SaveHeightMap(const std::string & folder_name) const {
            double one_x_line[y_grid +1]; 

            for(size_t i(0); i<(x_grid + 1); ++i){
                for(size_t k = 0; k<(y_grid+1); ++k){
                    one_x_line[k] = getHeight(dx*i + x_offset, dy*k + y_offset);
                    //printf("%lu %lu grid, (%lu, %lu), %f,%f\n", x_grid, y_grid, i, k, dx*i, dy*k);
                }
                saveVector(one_x_line, folder_name+"/HeightMap", y_grid+1, true);
            }
        }
};
#endif 
