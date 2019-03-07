#include "OptInterpreter.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <fstream>
#include <Utilities/Utilities_print.h>
#include <utilities/save_file.hpp>

template<typename T>
OptInterpreter<T>* OptInterpreter<T>::getOptInterpreter(){
    static OptInterpreter inter;
    return &inter;
}

template <typename T>
OptInterpreter<T>::OptInterpreter(){
    _start_loc.setZero();
    //_start_loc[0] = 0.2;
    //_start_loc[1] = 0.3;
}
template <typename T>
OptInterpreter<T>::~OptInterpreter(){
}

template<typename T>
void OptInterpreter<T>::updateBodyTarget(const T & t, 
        Vec3<T> & body_target, DVec<T> & body_vel, DVec<T> & body_acc){

    double pos[3]; pos[0] = 0.; pos[1] = 0.; pos[2] = 0.;
    double vel[3]; vel[0] = 0.; vel[1] = 0.; vel[2] = 0.;
    double acc[3]; acc[0] = 0.; acc[1] = 0.; acc[2] = 0.;

   
    _body_traj.getCurvePoint(t, pos);
    _body_traj.getCurveDerPoint(t, 1, vel);
    _body_traj.getCurveDerPoint(t, 2, acc);

    for(size_t i(0); i<3; ++i){
        body_target[i] = (pos[i] - _start_loc[i]);
        body_vel[i] = vel[i];
        body_acc[i] = acc[i];
    }

    //printf("time: %f\n", t);
    //pretty_print(body_target, std::cout, "body target");
    //pretty_print(body_vel, std::cout, "body vel");
    //pretty_print(body_acc, std::cout, "body acc");
}


template <typename T>
void OptInterpreter<T>::SetParameter(const std::string & setup_file){
    ParamHandler handler(setup_file);
    std::string str_tmp;
    handler.getString("OPT_walking_forward", "folder_name", str_tmp);

    std::string opt_result_file = THIS_COM"optimization/optimization_data/"+str_tmp+"/1_opt_result.txt";
    std::string foot_loc_file = THIS_COM"optimization/optimization_data/"+str_tmp+"/1_foot_loc.txt";

    std::ifstream f_opt_result(opt_result_file);
    std::ifstream f_foot_loc(foot_loc_file);
    std::string line;

    getline(f_opt_result, line);
    printf("Minimum cost: %s\n", line.c_str());

    getline(f_opt_result, line);
    std::istringstream iss(line);
    std::vector<double> x;
    double tmp;
    while(iss>>tmp){
        x.push_back(tmp);
    }
    //pretty_print(x, "opt result");

    int idx_offset_1(0);
    int idx_offset_2(9);
    std::vector<double> full_foot_pos;
    DVec<T> upcoming_step_pos(6);

    while(getline(f_foot_loc, line)){
        std::istringstream istr(line);
        while(istr>>tmp){
            full_foot_pos.push_back(tmp);
        }
        for(int i(0); i<3; ++i){
            upcoming_step_pos[i] = full_foot_pos[idx_offset_1 + i] - _start_loc[i];
        }
        for(int i(0); i<3; ++i){
            upcoming_step_pos[i+3] = full_foot_pos[idx_offset_2 + i] - _start_loc[i];
        }
        _foot_step_list.push_back(upcoming_step_pos);
        full_foot_pos.clear();

        if(idx_offset_1>0){
            idx_offset_1 = 0;
            idx_offset_2 = 9;
        } 
        else{
            idx_offset_1 = 3;
            idx_offset_2 = 6;
        } 
    }

    // Build up body trajectory
    buildSpline(x);
    buildOrientationSpline(x);
    printf("[Optimization Interpreter] Setting is done\n");
}

template<typename T>
void OptInterpreter<T>::buildSpline(const std::vector<double> & x){ 

    int idx_offset = 4*OptCase::_nStep;
    double init[9];
    double fin[9];

    int num_mid_pt = OptCase::_nStep-2;
    int degree = 1;

    init[0] = x[0 + idx_offset]; // starting loc
    init[1] = x[1 + idx_offset]; 
    init[2] = x[2 + idx_offset]; 

    init[3] = 0.; //velocity
    init[4] = 0.; 
    init[5] = 0.; 

    init[6] = 0.; 
    init[7] = 0.; //acceleration 
    init[8] = 0.; //acceleration 


    fin[0] = x[0 + idx_offset + 3*(num_mid_pt+1)]; // end loc
    fin[1] = x[1 + idx_offset + 3*(num_mid_pt+1)];
    fin[2] = x[2 + idx_offset + 3*(num_mid_pt+1)];

    fin[3] = 0.;
    fin[4] = 0.;
    fin[5] = 0.;

    fin[6] = 0.;
    fin[7] = 0.;
    fin[8] = 0.;

    // middle point allocation
    double ** middle_pt = new double*[OptCase::nMiddle_pt];
    for(int i(0); i<OptCase::nMiddle_pt; ++i) middle_pt[i] = new double[3];

    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            middle_pt[degree*i+j][0] = x[idx_offset + 3*(i+1)];
            middle_pt[degree*i+j][1] = x[idx_offset + 3*(i+1)+1];
            middle_pt[degree*i+j][2] = x[idx_offset + 3*(i+1)+2];
        }
    }
    _body_traj.SetParam(init, fin, middle_pt, OptCase::_tot_time);
    
    for(int i(0); i<OptCase::nMiddle_pt; ++i){
        delete [] middle_pt[i];
    }
    delete [] middle_pt;
}
#if (OriSplineDim == 2)
template<typename T>
void OptInterpreter<T>::updateBodyOriTarget(const T & t, Vec3<T> & body_ori){

    double ori[2]; ori[0] = 0.; ori[1] = 0.;
    _body_ori_traj.getCurvePoint(t, ori);

    for(size_t i(0); i<2; ++i){
        body_ori[i+1] = ori[i];
    }
    body_ori[0] = 0. ; // Zero Roll

    //printf("time: %f\n", t);
    pretty_print(body_ori, std::cout, "body ori pitch & yaw");
    //pretty_print(body_vel, std::cout, "body vel");
    //pretty_print(body_acc, std::cout, "body acc");
}
template<typename T>
void OptInterpreter<T>::buildOrientationSpline(const std::vector<double> & x){ 

    int idx_offset = 4*OptCase::_nStep + 3*OptCase::_nStep;
    double init[6];
    double fin[6];

    int num_mid_pt = OptCase::_nStep-2;
    int degree = 1;

    init[0] = x[0 + idx_offset]; // starting loc
    init[1] = x[1 + idx_offset]; 

    init[2] = 0.; //velocity
    init[3] = 0.; 

    init[4] = 0.; //acceleration 
    init[5] = 0.; //acceleration 


    fin[0] = x[0 + idx_offset + 2*(num_mid_pt+1)]; // end loc
    fin[1] = x[1 + idx_offset + 2*(num_mid_pt+1)];

    fin[2] = 0.;
    fin[3] = 0.;

    fin[4] = 0.;
    fin[5] = 0.;

    // middle point allocation
    double ** middle_pt_ori = new double*[OptCase::nMiddle_pt];
    for(int i(0); i<OptCase::nMiddle_pt; ++i) middle_pt_ori[i] = new double[2];

    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            middle_pt_ori[degree*i+j][0] = x[idx_offset + 2*(i+1)];
            middle_pt_ori[degree*i+j][1] = x[idx_offset + 2*(i+1)+1];
        }
    }
    _body_ori_traj.SetParam(init, fin, middle_pt_ori, OptCase::_tot_time);
    
    for(int i(0); i<OptCase::nMiddle_pt; ++i){
        delete [] middle_pt_ori[i];
    }
    delete [] middle_pt_ori;
}
#elif (OriSplineDim == 1)
template<typename T>
void OptInterpreter<T>::buildOrientationSpline(const std::vector<double> & x){ 

    int idx_offset = 4*OptCase::_nStep + 3*OptCase::_nStep;
    double init[3];
    double fin[3];

    int num_mid_pt = OptCase::_nStep-2;
    int degree = 1;

    init[0] = x[0 + idx_offset]; // starting loc
    init[1] = 0.; //velocity
    init[2] = 0.; //acceleration 


    fin[0] = x[0 + idx_offset + 1*(num_mid_pt+1)]; // end loc
    fin[1] = 0.;
    fin[2] = 0.;

    // middle point allocation
    double ** middle_pt_ori = new double*[OptCase::nMiddle_pt];
    for(int i(0); i<OptCase::nMiddle_pt; ++i) middle_pt_ori[i] = new double[1];

    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            middle_pt_ori[degree*i+j][0] = x[idx_offset + (i+1)];
        }
    }
    _body_ori_traj.SetParam(init, fin, middle_pt_ori, OptCase::_tot_time);
    
    for(int i(0); i<OptCase::nMiddle_pt; ++i){
        delete [] middle_pt_ori[i];
    }
    delete [] middle_pt_ori;
}

template<typename T>
void OptInterpreter<T>::updateBodyOriTarget(const T & t, Vec3<T> & body_ori){

    double ori[1]; ori[0] = 0.;
    _body_ori_traj.getCurvePoint(t, ori);

    body_ori[0] = 0. ; // Zero Roll
    body_ori[1] = ori[0];
    body_ori[2] = 0.;
    
    //printf("time: %f\n", t);
    //pretty_print(body_ori, std::cout, "body ori");
}
#endif




template class OptInterpreter<double>;
template class OptInterpreter<float>;
