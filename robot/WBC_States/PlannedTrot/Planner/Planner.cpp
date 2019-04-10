#include "Planner.hpp"
#include "Path.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Utilities/Utilities_print.h>
#include <Utilities/save_file.h>

template <typename T>
Planner<T>::Planner(int first_stance_foot):_step_idx(-1), 
    _first_stance_foot(first_stance_foot) {
    _curr_global_frame_loc.setZero();
    _sp = StateProvider<T>::getStateProvider();
}

template<typename T>
void Planner<T>::updateStepIdx(const T& curr_time){
    ++_step_idx;
    //printf("step idx: %d\n", _step_idx);
    _curr_path_st_time = curr_time;

    _path_flag = _engaged_path.checkIdx(_step_idx);
    if(_path_flag == path_flag::swap_call){
        //printf("original engage path\n");
        //_engaged_path.printPathInfo();
        //printf("replanned path\n");
        //_replanned_path.printPathInfo();
        
        // swap
        _engaged_path = _replanned_path;

        //printf("final path\n");
        _engaged_path.printPathInfo();
        
        _engaged_path._b_used = true;
        _curr_path_st_time = curr_time;
        _step_idx = 0;

        
        _curr_global_frame_loc.setZero();
        if(_engaged_path._stance_foot_list[0] == stance_foot::FRHL){
            _curr_global_frame_loc += 0.5 * _sp->_global_fr_loc;
            _curr_global_frame_loc += 0.5 * _sp->_global_hl_loc;
        }else if(_engaged_path._stance_foot_list[0] == stance_foot::FLHR){
            _curr_global_frame_loc += 0.5 * _sp->_global_fl_loc;
            _curr_global_frame_loc += 0.5 * _sp->_global_hr_loc;
        }

    }else if(_path_flag == path_flag::end_path){
        printf("end of path... stop until new user input comes\n");
        T body_lin[9];
        T body_ori[9];
        int stance_foot;
        _engaged_path.getPathInfo(_step_idx, 0.0 , body_lin, body_ori, stance_foot);
        _engaged_path.initialization(body_lin, body_ori, stance_foot);
        _replanned_path.initialization(body_lin, body_ori, stance_foot);
        _engaged_path._b_used = false;
        _step_idx = 0;
    }
}

template <typename T>
void Planner<T>::getNextFootLocation(int stance_foot, 
        Vec3<T> & front_foot, Vec3<T> & hind_foot){

    if(_engaged_path._stance_foot_list[_step_idx] != stance_foot){
        printf("[Error] %d th stance foot idx is not matched. request/return: %d/%d\n", 
                _step_idx, 
                stance_foot, 
                _engaged_path._stance_foot_list[_step_idx]);
    }
    
    _engaged_path.getSwingFootLoc(_step_idx, front_foot, hind_foot); 
}

template <typename T>
void Planner<T>::SetParameter(const std::string & config_file){
    ParamHandler handler(config_file);
    handler.getValue<T>("body_height", _body_height);
    handler.getValue<T>("min_walking_initiating_speed", _min_walking_initiating_speed);

    _engaged_path.setParameters(&handler);
    _replanned_path.setParameters(&handler);

    T lin_ini[9];
    T ang_ini[9];
    for(size_t i(0); i<9; ++i){
        lin_ini[i] = 0.;
        ang_ini[i] = 0.;
    }
    lin_ini[2] = _body_height;
    _engaged_path.initialization(lin_ini, ang_ini, _first_stance_foot);
}

template <typename T>
Planner<T>::~Planner(){}

template <typename T>
void Planner<T>::UpdateUserInput(T* dir, T* ori){

    static int replan_count(0);
    T user_command_size = fabs(dir[0]) + fabs(dir[1]) + fabs(ori[0]) + fabs(ori[1]) + fabs(ori[2]);
    if(user_command_size > 0.001){ // Update Path
        T ini_lin[9];
        T ini_ori[9];
        int ini_stance_foot;

        //printf("update user input\n");
        if(_engaged_path._b_used){
            _engaged_path._end_idx = _step_idx + 1;
            if(_step_idx<0){ 
                // If step is not initiated yet, 
                // then connect the next to the step after the current path
                _engaged_path._end_idx = 1;
            }
            //printf("end idx: %lu\n", _engaged_path._end_idx);
            //_engaged_path.printPathInfo();
            _engaged_path.getPathInfo(
                    _engaged_path._end_idx, 0., ini_lin, ini_ori, ini_stance_foot);

            ++replan_count;
           _replanned_path.updatePath(dir, ori, ini_lin, ini_ori, ini_stance_foot);
           _replanned_path._path_idx = replan_count;
           //_replanned_path.printPathInfo();
           _replanned_path._path_start_loc = _curr_global_frame_loc;
           for(size_t k(0); k<2; ++k) _replanned_path._path_start_loc[k] += ini_lin[k];
             //exit(0);
        }else{
            _engaged_path.getPathInfo(0, 0., ini_lin, ini_ori, ini_stance_foot);
            _engaged_path.updatePath(dir, ori, ini_lin, ini_ori, ini_stance_foot);
            // Simultaneously update next path target
            _replanned_path._fin_vel[0] = _engaged_path._fin_vel[0];
            _replanned_path._fin_vel[1] = _engaged_path._fin_vel[1];
            _replanned_path._fin_ori[2] = _engaged_path._fin_ori[2];

            _replanned_path._path_start_loc = _curr_global_frame_loc;
            for(size_t k(0); k<2; ++k) _replanned_path._path_start_loc[k] += ini_lin[k];
        }
    }
}

template <typename T>
void Planner<T>::getBodyConfig(const T & t, 
        Vec3<T>  & body_pos, Vec3<T>  & body_vel, Vec3<T> & body_acc, 
        Vec3<T>  & body_ori, Vec3<T>  & body_ang_vel ){

    T body_lin_info[9];
    T body_ang_info[9];
    int swing_foot;
    _engaged_path.getPathInfo(_step_idx, t - _curr_path_st_time, 
            body_lin_info, body_ang_info, swing_foot);

    for(size_t i(0); i<3; ++i){
        body_pos[i] =  body_lin_info[i];
        body_vel[i] =  body_lin_info[i + 3];
        body_acc[i] =  body_lin_info[i + 6];

        body_ori[i] = body_ang_info[i];
        body_ang_vel[i] = body_ang_info[i + 3];
    }
}

template<typename T>
void Planner<T>::updateExtraData(Cheetah_Extra_Data<T> * ext_data){
    size_t num_step = _engaged_path._end_idx + _replanned_path._end_idx;
    ext_data->num_step = num_step*2;
    T step_time = _engaged_path._step_time;

    // TODO drawing of path
    size_t resolution(50);
    ext_data->num_path_pt = resolution; 

    T dt = 1./((double)resolution);
    T pos[3];
    pos[0] =0.;
    pos[1] =0.;
    pos[2] =0.;
    for(size_t i(0); i<resolution; ++i){
        _engaged_path._lin_spline.getCurvePoint(dt * i, pos);
        ext_data->path_x[i] = pos[0];
        ext_data->path_y[i] = pos[1];
        ext_data->path_z[i] = pos[2];
    }

    // Foot location, arrows
    ext_data->num_middle_pt = num_step;
    T ori[3];
    ori[0] =0.;
    ori[1] =0.;
    ori[2] =0.;
   
    Vec3<T> previous_local_frame = _curr_global_frame_loc;
    Vec3<T> front_foot;
    Vec3<T> hind_foot;

    for(size_t i(0); i<num_step; ++i){
        if(i < _engaged_path._end_idx){
            _engaged_path._lin_spline.getCurvePoint(step_time* i, pos);
            _engaged_path._ori_spline.getCurvePoint(step_time* i, ori);
            pos[0] += _engaged_path._path_start_loc[0];
            pos[1] += _engaged_path._path_start_loc[1];
            pos[2] += _engaged_path._path_start_loc[2];

            _engaged_path.getSwingFootLoc(i-1, front_foot, hind_foot); 
            ext_data->loc_x[2*i] = front_foot[0] + previous_local_frame[0];
            ext_data->loc_y[2*i] = front_foot[1] + previous_local_frame[1];
            ext_data->loc_z[2*i] = front_foot[2] + previous_local_frame[2];
            
            ext_data->loc_x[2*i+ 1] = hind_foot[0] + previous_local_frame[0];
            ext_data->loc_y[2*i+ 1] = hind_foot[1] + previous_local_frame[1];
            ext_data->loc_z[2*i+ 1] = hind_foot[2] + previous_local_frame[2];
            
            previous_local_frame += 0.5 * front_foot;
            previous_local_frame += 0.5 * hind_foot;

        }else{
            size_t j = (i-_engaged_path._end_idx);
            _replanned_path._lin_spline.getCurvePoint(step_time* j, pos);
            _replanned_path._ori_spline.getCurvePoint(step_time* j, ori);

            pos[0] += _replanned_path._path_start_loc[0];
            pos[1] += _replanned_path._path_start_loc[1];
            pos[2] += _replanned_path._path_start_loc[2];

            _replanned_path.getSwingFootLoc(j-1, front_foot, hind_foot); 
            ext_data->loc_x[2*i] = front_foot[0] + previous_local_frame[0];
            ext_data->loc_y[2*i] = front_foot[1] + previous_local_frame[1];
            ext_data->loc_z[2*i] = front_foot[2] + previous_local_frame[2];
            
            ext_data->loc_x[2*i+ 1] = hind_foot[0] + previous_local_frame[0];
            ext_data->loc_y[2*i+ 1] = hind_foot[1] + previous_local_frame[1];
            ext_data->loc_z[2*i+ 1] = hind_foot[2] + previous_local_frame[2];
           
            previous_local_frame += 0.5 * front_foot;
            previous_local_frame += 0.5 * hind_foot;
        }
        ext_data->mid_ori_roll[i] = ori[0];
        ext_data->mid_ori_pitch[i] = ori[1];
        ext_data->mid_ori_yaw[i] = ori[2];

        ext_data->mid_x[i] = pos[0];
        ext_data->mid_y[i] = pos[1];
        ext_data->mid_z[i] = pos[2];
    }
}

template <typename T>
bool Planner<T>::stop(const T & curr_time){ 
    if(_step_idx<0){ _curr_path_st_time = curr_time; }
    // If the commanded path is too small to take a step, then stop

    T cmd_vel = sqrt(
            _engaged_path._fin_vel[0] * _engaged_path._fin_vel[0] + 
            _engaged_path._fin_vel[1] * _engaged_path._fin_vel[1]) 
        + fabs(_engaged_path._step_size_ori[2]);

    if(cmd_vel < _min_walking_initiating_speed
            && _engaged_path._step_size_ori[2] < _min_walking_initiating_speed){ 
        //printf("short command: %f/ %f\n", step_len, _min_trot_initiating_step_size);
        _step_idx = -1;
        _curr_path_st_time = curr_time;
        return true; 
    }
    // except that, keep walking
    _engaged_path._b_used = true;
    return false; 
}

template class Planner<double>;
template class Planner<float>;
