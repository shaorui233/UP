#include "Collision/CollisionPlane.h"


#include "Collision/CollisionPlane.h"

template<typename T>
bool CollisionPlane<T>::ContactDetection(
        const Vec3<T> & cp_pos, T & penetration, 
        Mat3<T> & cp_frame){

    if(cp_pos[2] < _height){
        penetration = cp_pos[2]-_height;
        cp_frame.setIdentity();
        return true;
    }else {
        return false; 
    }
}

template class CollisionPlane<double>;
template class CollisionPlane<float>;
