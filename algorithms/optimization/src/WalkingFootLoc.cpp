#include <WalkingFootLoc.hpp>
#include <stdio.h>
#include <Utilities/save_file.h>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <height_map/HeightMap.hpp>
#include <Math/orientation_tools.h>

double ** mid_pt_pos_foot_loc = new double*[WalkingFootLoc::nMiddle_pt];
double ** mid_pt_ori_foot_loc = new double*[WalkingFootLoc::nMiddle_pt];

WalkingFootLoc::WalkingFootLoc(){ 
    _intermittent_step_size = 50000;
    _hmap = NULL;
    for(int i(0); i<nMiddle_pt; ++i) mid_pt_pos_foot_loc[i] = new double[3];
    for(int i(0); i<nMiddle_pt; ++i) mid_pt_ori_foot_loc[i] = new double[WalkingFootLoc::dimOri];
}

WalkingFootLoc::~WalkingFootLoc(){
    for(int i(0); i<nMiddle_pt; ++i){
        delete [] mid_pt_pos_foot_loc[i];
        delete [] mid_pt_ori_foot_loc[i];
    }
    delete [] mid_pt_pos_foot_loc;
    delete [] mid_pt_ori_foot_loc;
}

void WalkingFootLoc::buildSpline(
        const std::vector<double> & x, 
        BS_Basic<double, 3, 3, nMiddle_pt, 2, 2> & pos_spline,
        BS_Basic<double, WalkingFootLoc::dimOri, 3, nMiddle_pt, 2, 2> & ori_spline){

    double init[9];
    double fin[9];

    int num_mid_pt = WalkingFootLoc::_nStep-2;
    int degree = 1;

    // Position
    int idx_offset = 4*WalkingFootLoc::_nStep;
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
    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            mid_pt_pos_foot_loc[degree*i+j][0] = x[idx_offset + 3*(i+1)];
            mid_pt_pos_foot_loc[degree*i+j][1] = x[idx_offset + 3*(i+1)+1];
            mid_pt_pos_foot_loc[degree*i+j][2] = x[idx_offset + 3*(i+1)+2];
        }
    }
    pos_spline.SetParam(init, fin, mid_pt_pos_foot_loc, _tot_time);

    // FootLoc
    double init_ori[3];
    double fin_ori[3];
    idx_offset = 4*WalkingFootLoc::_nStep + 3*WalkingFootLoc::_nStep;
    init_ori[0] = x[0 + idx_offset]; // starting loc
    init_ori[1] = 0.; //velocity
    init_ori[2] = 0.; //acceleration 


    fin_ori[0] = x[0 + idx_offset + 1*(num_mid_pt+1)]; // end loc
    fin_ori[1] = 0.;
    fin_ori[2] = 0.;

    // middle point allocation
    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            mid_pt_ori_foot_loc[degree*i+j][0] = x[idx_offset + (i+1)];
        }
    }
    ori_spline.SetParam(init_ori, fin_ori, mid_pt_ori_foot_loc, _tot_time);
}

void WalkingFootLoc::LandingMarginConstraint(unsigned m, double* result, unsigned n, 
        const double *x, double *grad, void* data){
    (void)m;
    (void)grad;
    (void)n;

    WalkingFootLoc* tester = (WalkingFootLoc*) data;
    HeightMap* hmap = tester->_hmap;
    double x_loc, y_loc, x_check, y_check;
    double landing_height;
    double height_var_sum(0.);

    double res(0.001);
    double check_region = tester->_landing_loc_region;
    int num_check = floor(check_region/res);
    for(int i(0); i<2*tester->_nStep; ++i){
        x_loc = x[2*i];
        y_loc = x[2*i+1];

        height_var_sum = 0.;
        landing_height = hmap->getHeight(x_loc, y_loc);

        for(int x_inc(0); x_inc<num_check; ++x_inc){
            for(int y_inc(0); y_inc<num_check; ++y_inc){
                x_check = x_loc + res*((double)x_inc) - check_region/2.;
                y_check = y_loc + res*((double)y_inc) - check_region/2.;
                height_var_sum += fabs(hmap->getHeight(x_check, y_check) - landing_height);
            }
        }
        height_var_sum /=num_check;
        height_var_sum /=num_check;
        result[i] = height_var_sum - tester->_landing_loc_height_var;

        //printf("num check:%d\n", num_check);
        //printf("x, y loc: %f, %f\n", x_loc, y_loc);
        //printf("landing height: %f, var sum: %f\n", landing_height, height_var_sum);
        //printf("result: %f\n", result[i]);
    }
}

void WalkingFootLoc::SaveOptimizationResult(
        const std::string& folder, const int & iter, 
        const double & opt_cost, const std::vector<double> & x, 
        const WalkingFootLoc* tester){

    std::string file_name = std::to_string(iter) + "_opt_result";
    saveValue(opt_cost, folder, file_name);
    // Raw optimization data save
    saveVector(x, folder, file_name);

    // time, body trajectory, foot stepping save
    BS_Basic<double, 3, 3, WalkingFootLoc::nMiddle_pt, 2, 2> body_traj;
    BS_Basic<double, WalkingFootLoc::dimOri, 3, WalkingFootLoc::nMiddle_pt, 2, 2> ori_traj;
    tester->buildSpline(x, body_traj, ori_traj);

    double dt(0.005);
    double curr_time(0.);
    double pos[3];
    double vel[3];
    double acc[3];

    double ori[WalkingFootLoc::dimOri];
    double ang_vel[WalkingFootLoc::dimOri];
    double ang_acc[WalkingFootLoc::dimOri];
    while(curr_time < (_tot_time+dt) ){
        body_traj.getCurvePoint(curr_time, pos);
        body_traj.getCurveDerPoint(curr_time, 1, vel);
        body_traj.getCurveDerPoint(curr_time, 2, acc);

        saveVector(pos, folder, std::to_string(iter)+"_body_pos", 3);
        saveVector(vel, folder, std::to_string(iter)+"_body_vel", 3);
        saveVector(acc, folder, std::to_string(iter)+"_body_acc", 3);


        ori_traj.getCurvePoint(curr_time, ori);
        ori_traj.getCurveDerPoint(curr_time, 1, ang_vel);
        ori_traj.getCurveDerPoint(curr_time, 2, ang_acc);

        saveVector(ori, folder, std::to_string(iter)+"_body_ori", WalkingFootLoc::dimOri);
        saveVector(ang_vel, folder, std::to_string(iter)+"_body_ang_vel", WalkingFootLoc::dimOri);
        saveVector(ang_acc, folder, std::to_string(iter)+"_body_ang_acc", WalkingFootLoc::dimOri);

        saveValue(curr_time, folder, std::to_string(iter)+"_time");

        curr_time += dt;
    }
    
    // (num_step-1) X (3*4)
    std::vector< std::vector<double> > FootLoc;

    buildFootStepLocation(x, FootLoc, tester->_hmap);

    for(int i(0); i < (WalkingFootLoc::_nStep -1); ++i){
        saveVector(FootLoc[i], folder, std::to_string(iter)+"_foot_loc");
    }
}

void WalkingFootLoc::buildFootStepLocation(const std::vector<double> & x, 
        std::vector<std::vector<double>> & foot_loc_list, const HeightMap* hmap){

    std::vector<double> foot_loc(12); // num leg x (x, y, z)
    foot_loc_list.resize(WalkingFootLoc::_nStep -1);

    int fr_idx, fl_idx, hr_idx, hl_idx;
    for(unsigned int i(0); i<(WalkingFootLoc::_nStep -1); ++i){
        if(i%2 == 0) {
            fr_idx = 4*i + 4;
            fl_idx = 4*i;
            hr_idx = 4*i + 2;
            hl_idx = 4*i + 6;
        }else {
            fr_idx = 4*i;
            fl_idx = 4*i+4;
            hr_idx = 4*i+6;
            hl_idx = 4*i+2;
        }
        // FR
        foot_loc[0] = x[fr_idx];
        foot_loc[1] = x[fr_idx+1];
        foot_loc[2] = hmap->getHeight(foot_loc[0], foot_loc[1]);
        // FL
        foot_loc[3] = x[fl_idx];
        foot_loc[4] = x[fl_idx + 1];
        foot_loc[5] = hmap->getHeight(foot_loc[3], foot_loc[4]);
        // HR
        foot_loc[6] = x[hr_idx];
        foot_loc[7] = x[hr_idx+1];
        foot_loc[8] = hmap->getHeight(foot_loc[6], foot_loc[7]);
        // HL
        foot_loc[9] =  x[hl_idx];
        foot_loc[10] = x[hl_idx+1];
        foot_loc[11] = hmap->getHeight(foot_loc[9], foot_loc[10]);

        foot_loc_list[i] = foot_loc;
    }
}

void WalkingFootLoc::InitialFinalConstraint(
        unsigned m, double * result, unsigned n, const double *x, 
        double * grad, void*data){

    (void)m;
    (void)n;
    (void)grad;
    WalkingFootLoc* test = (WalkingFootLoc*) data;

    for(int i(0); i<2; ++i){
        result[i] = x[i]- test->_ini_front_foot_loc[i];
        result[i+2] = x[i+2]- test->_ini_hind_foot_loc[i];

        result[i+4] = x[i+4*(test->_nStep-2)] - test->_fin_fr_loc[i];
        result[i+6] = x[i+4*(test->_nStep-2)+2] - test->_fin_hl_loc[i];
        result[i+8] = x[i+4*(test->_nStep-1)] - test->_fin_fl_loc[i];
        result[i+10] = x[i+4*(test->_nStep-1)+2] - test->_fin_hr_loc[i];

    }
}

void WalkingFootLoc::_SetInitialGuess(std::vector<double> & x){
    // Foot initial and Final
    for(int i(0); i<2; ++i){
        x[i] = _ini_front_foot_loc[i];
        x[i+2] =  _ini_hind_foot_loc[i];
        x[i+4*(_nStep-2)] = _fin_fr_loc[i];
        x[i+4*(_nStep-2)+2] = _fin_hl_loc[i];
        x[i+4*(_nStep-1)] = _fin_fl_loc[i];
        x[i+4*(_nStep-1)+2] = _fin_hr_loc[i];
    }

    double inc_foot = (_fin_fr_loc[0] - _ini_front_foot_loc[0])/(_nStep-2);
    for(int i(1); i<_nStep-2; ++i){
        x[4*i] = x[4*(i-1)] + inc_foot;
        x[4*i+1] = x[4*(i-1)+3];

        x[4*i+2] = x[4*(i-1)+2] + inc_foot;
        x[4*i+3] = x[4*(i-1)+1];
    }

    // Body Pos Initial and Final
    for(int i(0); i<3; ++i){
        x[i+idx_offset] = _ini_body_pos[i];
        x[i+idx_offset + 3*(_nStep-1)] = _fin_body_pos[i];
    }

    double delta_progress = (_fin_body_pos[0]-_ini_body_pos[0])/(_nStep-1);

    std::vector< std::vector<double> > FootLoc;
    buildFootStepLocation(x, FootLoc, _hmap);
    double forward_z, hind_z;


    for(int i(1); i<_nStep-1; ++i){
        x[3*i + idx_offset] = _ini_body_pos[0] + delta_progress*i;
        x[3*i + idx_offset + 1] = _ini_body_pos[1];

        // Height based on Foot height
        forward_z = (FootLoc[i][2] + FootLoc[i][5])/2.;
        hind_z = (FootLoc[i][8] + FootLoc[i][11])/2.;
        x[3*i + idx_offset + 2] = _ini_body_pos[2] + (forward_z + hind_z)/2.;
 
        //x[3*i + idx_offset + 2] = _ini_body_pos[2] + 
            //_hmap->getHeight(_ini_body_pos[0] + delta_progress*i, _ini_body_pos[1]);
    }

    // Orientation set by zero
    x[idx_offset + 3*_nStep] = _ini_body_ori;
    for(int i(1); i<_nStep-1; ++i){
        x[idx_offset + 3*_nStep + i] = 0.;
    }
    x[idx_offset + 3*_nStep + _nStep - 1] = _fin_body_ori;
}

void WalkingFootLoc::nice_print_result(const std::vector<double> & x){
    for(int i(0);i<_nStep; ++i){
        printf("%d th step: (%f, %f), (%f, %f)\n", i,
                x[4*i], x[4*i+1], x[4*i+2], x[4*i+3]);
    }

    int idx_offset = 4*_nStep;
    for(int i(0); i<_nStep; ++i){
        printf("%d th body: (%f, %f, %f)\n", i,
                x[3*i + idx_offset], x[3*i + idx_offset +1],
                x[3*i + idx_offset +2]);
    }

    idx_offset = 4*_nStep + 3*_nStep;
    for(int i(0); i<_nStep; ++i){
        printf("%d th body ori (pitch): %f\n", i,
                x[i + idx_offset]);
    }
}
void WalkingFootLoc::_getRotationMatrix(const double & pitch, Mat3<double> & rot) const{
    double cy = std::cos(pitch);
    double sy = std::sin(pitch);

    rot << cy, 0, sy,
        0, 1, 0,
        -sy, 0, cy;
}
