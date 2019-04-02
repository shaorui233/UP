#include "Path.hpp"
#include <Math/orientation_tools.h>
#include <Utilities/Utilities_print.h>
#include <Utilities/save_file.h>

template <typename T>
Path<T>::Path(): _path_idx(0){
    _b_used = false;
    _end_idx = 0;
    _step_size_min.clear(); _step_size_min.resize(2, 0.);
    _step_size_max.clear(); _step_size_max.resize(2, 0.);

    _front_footloc_list.resize(nMiddle + 2);
    _hind_footloc_list.resize(nMiddle + 2);
    _stance_foot_list.resize(nMiddle+3);
 
    mid_ori = new T*[nMiddle];
    mid_lin = new T*[nMiddle];
    
    for(size_t i(0); i< nMiddle; ++i){
        mid_ori[i] = new T[3];
        mid_lin[i] = new T[3];
    }

    for(size_t i(0); i<9; ++i){
        _fin_ori[i] = 0.;
        _fin_lin[i] = 0.;
    }

    _path_start_loc.setZero();
}

template<typename T>
void Path<T>::getSwingFootLoc(const size_t & idx, Vec3<T> & front_foot, Vec3<T> & hind_foot){
    Vec3<T> local_frame; local_frame.setZero();

    local_frame += 0.5 * _front_footloc_list[idx];
    local_frame += 0.5 * _hind_footloc_list[idx];

    front_foot = _front_footloc_list[idx+1] - local_frame;
    hind_foot = _hind_footloc_list[idx+1] - local_frame;

    //printf("%lu th:\n", idx);
    //pretty_print(local_frame, std::cout, "local frame");
    //pretty_print(front_foot, std::cout, "front foot");
    //pretty_print(hind_foot, std::cout, "hind foot");
}

template <typename T>
int Path<T>::checkIdx(const int & step_idx){

    // If the step idx is equal or larger than end of path
    if(step_idx >= (int)_end_idx){
        if(step_idx >= (nMiddle + 1)){ // If there is no more path
            return path_flag::end_path;
        }
        return path_flag::swap_call;
    }
    return path_flag::step;
}

template <typename T>
Path<T>::~Path(){
    for(size_t i(0); i<nMiddle; ++i){
        delete[] mid_ori[i];
        delete[] mid_lin[i];
    }
    delete[] mid_ori;
    delete[] mid_lin;
}

template<typename T>
void Path<T>::initialization(T* ini_lin, T* ini_ori, int stance_foot){
    _b_used = false;
    
    _step_size[0] = 0.;
    _step_size[1] = 0.;
    
    _step_size_ori[0] = 0.;
    _step_size_ori[1] = 0.;
    _step_size_ori[2] = 0.;
    
    _end_idx = nMiddle+1;

    T dir[2]; dir[0] = 0.; dir[1] = 0.;
    T ori[3]; ori[0] = 0.; ori[1] = 0.; ori[2] = 0.;
    updatePath(dir, ori, ini_lin, ini_ori, stance_foot);
}

template<typename T>
void Path<T>::getPathInfo(const int & idx, const T & t, 
        T* body_lin, T* body_ori, int & stance_foot){
    T check_time = ((T)idx) * _step_time + t;

    if(idx < 0){ check_time = 0.; }

    T pos[3]; pos[0] =0.; pos[1] =0.; pos[2] =0.;
    T vel[3]; vel[0] =0.; vel[1] =0.; vel[2] =0.;
    T acc[3]; acc[0] =0.; acc[1] =0.; acc[2] =0.;

    T ori[3]; ori[0] =0.; ori[1] =0.; ori[2] =0.;
    T ang_vel[3]; ang_vel[0] =0.; ang_vel[1] =0.; ang_vel[2] =0.;
    T ang_acc[3]; ang_acc[0] =0.; ang_acc[1] =0.; ang_acc[2] =0.;

    _lin_spline.getCurvePoint(check_time, pos);
    _lin_spline.getCurveDerPoint(check_time, 1, vel);
    _lin_spline.getCurveDerPoint(check_time, 2, acc);

    _ori_spline.getCurvePoint(check_time, ori);
    _ori_spline.getCurveDerPoint(check_time, 1, ang_vel);
    _ori_spline.getCurveDerPoint(check_time, 2, ang_acc);

    saveValue(check_time, "robot/WBC_States/sim_data/", "check_time");
    saveVector(vel, "robot/WBC_States/sim_data/", "check_vel", 3);
    for(size_t i(0); i<3; ++i){
        body_lin[i] = pos[i];
        body_lin[i + 3] = vel[i];
        body_lin[i + 6] = acc[i];

        body_ori[i] = ori[i];
        body_ori[i + 3] = ang_vel[i];
        body_ori[i + 6] = ang_acc[i];
    }
    stance_foot = _stance_foot_list[idx];

    Vec3<T> local_frame; local_frame.setZero();
    local_frame += 0.5 * _front_footloc_list[idx];
    local_frame += 0.5 * _hind_footloc_list[idx];

    body_lin[0] -= local_frame[0];
    body_lin[1] -= local_frame[1];
    body_lin[2] -= local_frame[2];
}


template <typename T>
bool Path<T>::checkEnd(const size_t & idx){
    if(idx >= _end_idx){
        return true;
    }
    return false;
}
template <typename T>
void Path<T>::updatePath(T* dir, T* ori, T* ini_lin, T* ini_ori, int ini_stance_foot){
    // Update User Input 
    // (Currently it only take yaw rotation command)
    // Currently robot can walk on flat terrain
    _end_idx = nMiddle + 1;
    
    // Orientation
    _step_size_ori[2] += (-0.0005*ori[2]);
    for(size_t i(0); i<3; ++i){
        _step_size_ori[i] = _bound(_step_size_ori[i], _step_size_ori_min[i], _step_size_ori_max[i]);
        mid_ori[0][i] = ini_ori[i] + _step_size_ori[i];
    }

    for(size_t i(1); i< nMiddle; ++i){
        for(size_t j(0); j<3; ++j){
            mid_ori[i][j] = _step_size_ori[j] + mid_ori[i-1][j];
        }
    }
    for(size_t i(0); i<3; ++i){
        _fin_ori[i] = _step_size_ori[i] + mid_ori[nMiddle-1][i];
    }
    _ori_spline.SetParam(ini_ori, _fin_ori, mid_ori, _tot_time);
 
    // Linear
    _step_size[0] += (0.0005*dir[0]);
    _step_size[1] += (-0.001*dir[1]);
    
    for(size_t i(0); i<2; ++i)
        _step_size[i] = _bound(_step_size[i], _step_size_min[i], _step_size_max[i]);

    mid_lin[0][0] = ini_lin[0] + _step_size[0] * cos(ini_ori[2]);
    mid_lin[0][1] = ini_lin[1] + _step_size[1] * sin(ini_ori[2]);
    mid_lin[0][2] = ini_lin[2];

    T yaw;
    for(size_t i(1); i< nMiddle; ++i){
        yaw = mid_ori[i-1][2];
        mid_lin[i][0] = cos(yaw) * _step_size[0] - sin(yaw) * _step_size[1] + mid_lin[i-1][0];
        mid_lin[i][1] = sin(yaw) * _step_size[0] + cos(yaw) * _step_size[1] + mid_lin[i-1][1];
        mid_lin[i][2] = ini_lin[2];
    }
    yaw = mid_ori[nMiddle-1][2];
    _fin_lin[0] = cos(yaw) * _step_size[0] - sin(yaw) * _step_size[1] + mid_lin[nMiddle-1][0];
    _fin_lin[1] = sin(yaw) * _step_size[0] + cos(yaw) * _step_size[1] + mid_lin[nMiddle-1][1];
    _fin_lin[2] = ini_lin[2];

    _lin_spline.SetParam(ini_lin, _fin_lin, mid_lin, _tot_time);
    
    // Foot Location
    _updateStepLocation(ini_stance_foot);
}

template<typename T>
void Path<T>::_updateStepLocation(const int & ini_stance_foot){
   // Step location based on full contact configuration
    T pos[3]; pos[0] =0.; pos[1] =0.; pos[2] =0.;
    T vel[3]; vel[0] =0.; vel[1] =0.; vel[2] =0.;
    T ori[3]; ori[0] =0.; ori[1] =0.; ori[2] =0.;
    T ang_vel[3]; ang_vel[0] =0.; ang_vel[1] =0.; ang_vel[2] =0.;
     
    Vec3<T> body_pos, body_vel, body_ori_rpy, body_ang_vel;
    Vec3<T> front_foot, hind_foot;

    front_foot.setZero();
    hind_foot.setZero();
    _stance_foot_list[0] = ini_stance_foot;
    Mat3<T> Rot, Rot_trans;
    for(size_t i(0); i< nMiddle + 2; ++i){
        // Get full contact phase body pos/ori
        T time = _step_time * (i);
        _lin_spline.getCurvePoint(time, pos);
        _lin_spline.getCurveDerPoint(time, 1, vel);
        _ori_spline.getCurvePoint(time, ori);
        _ori_spline.getCurveDerPoint(time, 1, ang_vel);

        // Build Vector 
        for(size_t j(0); j<3; ++j){
            body_pos[j] = pos[j];
            body_vel[j] = vel[j];
            body_ori_rpy[j] = ori[j];
            body_ang_vel[j] = ang_vel[j];
        }
        Rot_trans = ori::rpyToRotMat(body_ori_rpy);
        Rot = Rot_trans.transpose();

        // Swing foot list
        if(_stance_foot_list[i] == stance_foot::FRHL){
            computeFootLoc(Rot, _fr_shoulder, _step_time, body_pos, body_vel, body_ang_vel, front_foot);
            computeFootLoc(Rot, _hl_shoulder, _step_time, body_pos, body_vel, body_ang_vel, hind_foot);
        }
        if(_stance_foot_list[i] == stance_foot::FLHR){
            computeFootLoc(Rot, _fl_shoulder, _step_time, body_pos, body_vel, body_ang_vel, front_foot);
            computeFootLoc(Rot, _hr_shoulder, _step_time, body_pos, body_vel, body_ang_vel, hind_foot);
        }
        _front_footloc_list[i] = front_foot;
        _hind_footloc_list[i] = hind_foot;
        _stance_foot_list[i+1] = -_stance_foot_list[i];

        //printf("[Update Step Location]\n");
        //printf("curr time: %f, %d\n", time, _stance_foot_list[i]);
        //pretty_print(body_pos, std::cout, "body pos");        
        //pretty_print(body_vel, std::cout, "body vel");        
        //pretty_print(body_ori_rpy, std::cout, "body ori");        
        //pretty_print(body_ang_vel, std::cout, "body ang vel");        
        //pretty_print(front_foot, std::cout, "front foot");
        //pretty_print(hind_foot, std::cout, "hind foot");
    }
}

template<typename T>
void Path<T>::computeFootLoc(const Mat3<T> & Rot, const Vec3<T> & shoulder, 
        const T& step_time, 
        const Vec3<T> & body_pos, const Vec3<T> & body_vel, 
        const Vec3<T> & body_ang_vel, Vec3<T> & foot_loc){

    foot_loc = body_pos + Rot * shoulder + 
        step_time/2. *(body_vel + body_ang_vel.cross(Rot * shoulder));
    foot_loc += body_pos[2]/ 9.81 * body_vel.cross(body_ang_vel);
    foot_loc[2] = 0.;
}

template<typename T>
void Path<T>::setParameters(ParamHandler* handle){
    // Timing Parameter
    _step_time = 0.;
    T t;
    handle->getValue<T>("swing_time", t);
    _step_time += t;
    handle->getValue<T>("transition_time", t);
    _step_time += (2.*t);
    handle->getValue<T>("stance_time", t);
    _step_time += t;
    _tot_time = _step_time * (T)(nMiddle+1);

    // Limit
    handle->getVector<T>("step_size_min", _step_size_min);
    handle->getVector<T>("step_size_max", _step_size_max);
    handle->getVector<T>("step_size_ori_min", _step_size_ori_min);
    handle->getVector<T>("step_size_ori_max", _step_size_ori_max);


    // Body size
    T half_depth, half_width;
    handle->getValue<T>("body_half_depth", half_depth);
    handle->getValue<T>("body_half_width", half_width);

    
    _fr_shoulder[0] = half_depth;
    _fr_shoulder[1] = -half_width;
    _fr_shoulder[2] = 0.;

    _fl_shoulder[0] = half_depth;
    _fl_shoulder[1] = half_width;
    _fl_shoulder[2] = 0.;
  
    _hr_shoulder[0] = -half_depth;
    _hr_shoulder[1] = -half_width;
    _hr_shoulder[2] = 0.;

    _hl_shoulder[0] = -half_depth;
    _hl_shoulder[1] = half_width;
    _hl_shoulder[2] = 0.;
}

template<typename T>
T Path<T>::_bound(const T & value, const T & min, const T & max){
    if(value > max){ return max; }
    if(value < min){ return min; }
    return value;
}
template<typename T>
void Path<T>::printPathInfo(){

    printf("path idx, end idx, step size: %d, %lu, (%f, %f)\n",
            _path_idx, 
            _end_idx, _step_size[0], _step_size[1]);
    printf("step time, total time: %f, %f\n", _step_time, _tot_time);

    T pos[3]; T vel[3]; T ori[3];
    for(size_t i(0); i<nMiddle+1; ++i){
        _lin_spline.getCurvePoint(_step_time*i, pos);
        _lin_spline.getCurveDerPoint(_step_time*i, 1, vel);
        _ori_spline.getCurvePoint(_step_time*i, ori);
        printf("%lu th body pos: %f, %f, %f\n", i, pos[0], pos[1], pos[2]);
        printf("%lu th body vel: %f, %f, %f\n", i, vel[0], vel[1], vel[2]);
        printf("%lu th body ori: %f, %f, %f\n", i, ori[0], ori[1], ori[2]);

        printf("%lu th stance foot idx: %d\n", i, _stance_foot_list[i]); 
        printf("%lu th front foot loc: %f, %f, %f\n", i, 
                _front_footloc_list[i][0],
                _front_footloc_list[i][1],
                _front_footloc_list[i][2]);

        printf("%lu th hind foot loc: %f, %f, %f\n", i, 
                _hind_footloc_list[i][0],
                _hind_footloc_list[i][1],
                _hind_footloc_list[i][2]);
        printf("\n");
     }
}
template class Path<double>;
template class Path<float>;

