#include "Planner.hpp"

#include <ParamHandler/ParamHandler.hpp>

template <typename T>
Path<T>::Path(){}

template <typename T>
Path<T>::~Path(){}

template class Path<double>;
template class Path<float>;

template <typename T>
void Path<T>::append(size_t insert_idx){
    _end_idx = insert_idx;
}

/* ================  End of Path class ============================ */

template <typename T>
Planner<T>::Planner(){
    // (x, y)
    _step_size[0] = 0.;
    _step_size[1] = 0.;
    for(size_t i(0); i<9;  ++i){ 
        _ini_pos[i] = 0.;
        _ini_ori[i] = 0.;
        _fin_pos[i] = 0.;
        _fin_ori[i] = 0.;
    }
}
template <typename T>
void Planner<T>::SetParameter(const std::string & config_file){
    ParamHandler handler(config_file);
    // Timing Parameter
    _step_time = 0.;
    T t;
    handler.getValue<T>("swing_time", t);
    _step_time += t;
    handler.getValue<T>("transition_time", t);
    _step_time += (2.*t);
    handler.getValue<T>("stance_time", t);
    _step_time += t;
    _tot_time = _step_time * (double)(nMiddle+1);

    // Limit
    handler.getVector<T>("step_size_min", _step_size_min);
    handler.getVector<T>("step_size_max", _step_size_max);
}

template <typename T>
Planner<T>::~Planner(){}

template <typename T>
void Planner<T>::UpdateUserInput(T* dir, T* ori){
    // Update User Input
    _fin_ori[2] += (-0.001*ori[2]);

    T step_size_roll = (_fin_ori[0] - _ini_ori[0])/(nMiddle + 1.);
    T step_size_pitch = (_fin_ori[1] - _ini_ori[1])/(nMiddle + 1.);
    T step_size_yaw = (_fin_ori[2] - _ini_ori[2])/(nMiddle + 1.);

    T** mid_ori = new T*[nMiddle];
    for(size_t i(0); i< nMiddle; ++i){
        mid_ori[i] = new T[3];

        mid_ori[i][0] = _ini_ori[0] + step_size_roll * i;
        mid_ori[i][1] = _ini_ori[1] + step_size_pitch * i;
        mid_ori[i][2] = _ini_ori[2] + step_size_yaw * i;
    }
    _ori_spline.SetParam(_ini_ori, _fin_ori, mid_ori, _tot_time);
 
    _step_size[0] += (0.0005*dir[0]);
    _step_size[1] += (-0.0002*dir[1]);

    
    for(size_t i(0); i<2; ++i)
        _step_size[i] = _bound(_step_size[i], _step_size_min[i], _step_size_max[i]);

    T** mid_pos = new T*[nMiddle];

    mid_pos[0] = new T[3];
    mid_pos[0][0] = _ini_pos[0] + _step_size[0] * cos(_ini_ori[2]);
    mid_pos[0][1] = _ini_pos[1] + _step_size[1] * sin(_ini_ori[2]);
    mid_pos[0][2] = 0.;

    T yaw;
    for(size_t i(1); i< nMiddle; ++i){
        mid_pos[i] = new T[3];

        yaw = mid_ori[i-1][2];
        mid_pos[i][0] = cos(yaw) * _step_size[0] - sin(yaw) * _step_size[1] + mid_pos[i-1][0];
        mid_pos[i][1] = sin(yaw) * _step_size[0] + cos(yaw) * _step_size[1] + mid_pos[i-1][1];
        mid_pos[i][2] = 0.;
    }
    yaw = mid_ori[nMiddle-1][2];
    _fin_pos[0] = cos(yaw) * _step_size[0] - sin(yaw) * _step_size[1] + mid_pos[nMiddle-1][0];
    _fin_pos[1] = sin(yaw) * _step_size[0] + cos(yaw) * _step_size[1] + mid_pos[nMiddle-1][1];

    _pos_spline.SetParam(_ini_pos, _fin_pos, mid_pos, _tot_time);

    for(size_t i(0); i<nMiddle; ++i){
        delete mid_ori[i];
        delete mid_pos[i];
    }
    delete mid_ori;
    delete mid_pos;
 
}

template <typename T>
void Planner<T>::getBodyPosture(const T & time, DVec<T>  & body_pos, DVec<T>  & body_ori ){
    T pos[3];
    pos[0] =0.;
    pos[1] =0.;
    pos[2] =0.;
 
    T ori[3];
    ori[0] =0.;
    ori[1] =0.;
    ori[2] =0.;
 

    _pos_spline.getCurvePoint(time, pos);
    _ori_spline.getCurvePoint(time, ori);

    for(size_t i(0); i<3; ++i){
        body_pos[i] = pos[i];
        body_ori[i] = ori[i];
    }

}

template <typename T>
T Planner<T>::_bound(const T & value, const T & min, const T & max){
    if(value > max){ return max; }
    if(value < min){ return min; }
    return value;
}

template <typename T>
bool Planner<T>::stop(){ return true; }
template class Planner<double>;
template class Planner<float>;
