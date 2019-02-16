#include "Collision/CollisionMesh.h"

/*!
 * check whether the contact happens or not
 * cp_frame let you know which direction is normal and which directions are 
 * x and y. The frame is basically contact coordinate w.r.t global. 
 */
template<typename T>
bool CollisionMesh<T>::ContactDetection(
        const Vec3<T> & cp_pos, T& penetration, Mat3<T> & cp_frame){

    // Contact detection
    Vec3<T> cp_pos_in_height_map = cp_pos - _left_corner_loc;
    if( (0<cp_pos_in_height_map[0]) && (cp_pos_in_height_map[0]<_x_max) ){
        if( (0<cp_pos_in_height_map[1]) && (cp_pos_in_height_map[1]<_y_max) ){
            int x_idx = floor(cp_pos_in_height_map[0]/_grid);
            int y_idx = floor(cp_pos_in_height_map[1]/_grid);

            T height_ave = 0.25 * _height_map(x_idx, y_idx) 
                + 0.25 * _height_map(x_idx, y_idx+1)
                + 0.25 * _height_map(x_idx+1, y_idx)
                + 0.25 * _height_map(x_idx+1, y_idx+1);

            if(cp_pos_in_height_map[2] < height_ave){
                penetration = cp_pos_in_height_map[2] - height_ave;
                // TODO cp frame computation
                cp_frame.setIdentity();
                return true;
            }
        }
    }


    return false;
}

template class CollisionMesh<double>;
template class CollisionMesh<float>;
