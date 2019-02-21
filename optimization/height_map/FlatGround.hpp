#ifndef FLAT_GROUND_H
#define FLAT_GROUND_H

#include "HeightMap.hpp"

class FlatGround: public HeightMap{
    public:
        FlatGround():HeightMap(){}
        virtual ~FlatGround(){}

        virtual double getHeight(const double & x, const double & y) const {
            (void)x;
            (void)y;

            return 0.;
        }
};
#endif
