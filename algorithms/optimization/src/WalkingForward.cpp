#include <WalkingForward.hpp>
#include <stdio.h>
#include <utilities/save_file.hpp>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <height_map/SeparateTerrain.hpp>
#include <height_map/FlatGround.hpp>
#include <height_map/RandomTerrain.hpp>
#include <time.h>

double ** middle_pt = new double*[WalkingForward::nMiddle_pt];

WalkingForward::WalkingForward(){ 
    _intermittent_step_size = 50000;
    _hmap = NULL;
    for(int i(0); i<nMiddle_pt; ++i) middle_pt[i] = new double[3];
}

WalkingForward::~WalkingForward(){
    for(int i(0); i<nMiddle_pt; ++i){
        delete [] middle_pt[i];
    }
    delete [] middle_pt;
}

void WalkingForward::buildSpline(
        const std::vector<double> & x, 
        BS_Basic<double, 3, 3, nMiddle_pt, 2, 2> & spline){

    int idx_offset = 4*WalkingForward::_nStep;
    double init[9];
    double fin[9];

    int num_mid_pt = WalkingForward::_nStep-2;
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
    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            middle_pt[degree*i+j][0] = x[idx_offset + 3*(i+1)];
            middle_pt[degree*i+j][1] = x[idx_offset + 3*(i+1)+1];
            middle_pt[degree*i+j][2] = x[idx_offset + 3*(i+1)+2];
        }
    }
    spline.SetParam(init, fin, middle_pt, _tot_time);
}

// TODO
void WalkingForward::KinematicsConstraint(unsigned m, double* result, unsigned n, 
        const double *x, double *grad, void* data){

    (void)m;
    (void)grad;

    WalkingForward* tester = (WalkingForward*) data;
    double d = tester->_half_body_length;
    double w = tester->_half_body_width;
    double swing_time = tester->_onestep_duration;
    HeightMap* hmap = tester->_hmap;
    BS_Basic<double, 3, 3, WalkingForward::nMiddle_pt, 2, 2> body_traj;

    std::vector<double> x_vec(n);
    for(unsigned int i(0); i<n; ++i){ x_vec[i] = x[i]; }
    tester->buildSpline(x_vec, body_traj);
    double body_pos[3]; body_pos[0] = 0.; body_pos[1] = 0.; body_pos[2] = 0.;
    double p0[3], p1[3], p2[3], p3[3]; //Fr, Fl, Hr, Hl
    double dx, dy, dz;
    double leg_length;

    double min = tester->_min_leg_length;
    double max = tester->_max_leg_length;

    for(int i(1); i<tester->_nStep; ++i){
        double time = i *swing_time;
        body_traj.getCurvePoint(time, body_pos);

        if(i%2 == 1){
            p0[0] = x[4*i];
            p0[1] = x[4*i + 1];
            p0[2] = hmap->getHeight(p0[0], p0[1]);

            p1[0] = x[4*(i-1)];
            p1[1] = x[4*(i-1) + 1];
            p1[2] = hmap->getHeight(p1[0], p1[1]);

            p2[0] = x[4*(i-1) + 2];
            p2[1] = x[4*(i-1) + 3];
            p2[2] = hmap->getHeight(p2[0], p2[1]);

            p3[0] = x[4*(i) + 2];
            p3[1] = x[4*(i) + 3];
            p3[2] = hmap->getHeight(p3[0], p3[1]);

        }else {
            p0[0] = x[4*(i-1)];
            p0[1] = x[4*(i-1) + 1];
            p0[2] = hmap->getHeight(p0[0], p0[1]);


            p1[0] = x[4*(i)];
            p1[1] = x[4*(i) + 1];
            p1[2] = hmap->getHeight(p1[0], p1[1]);


            p2[0] = x[4*(i) + 2];
            p2[1] = x[4*(i) + 3];
            p2[2] = hmap->getHeight(p2[0], p2[1]);


            p3[0] = x[4*(i-1) + 2];
            p3[1] = x[4*(i-1) + 3];
            p3[2] = hmap->getHeight(p3[0], p3[1]);
        }

        // FR
        dx = (body_pos[0] + d - p0[0]);
        dy = (body_pos[1] - w - p0[1]);
        dz = (body_pos[2] - p0[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        result[8*(i-1)] = min - leg_length;
        result[8*(i-1) + 1] = leg_length - max;

        // FL
        dx = (body_pos[0] + d - p1[0]);
        dy = (body_pos[1] + w - p1[1]);
        dz = (body_pos[2] - p1[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        result[8*(i-1) + 2] = min - leg_length;
        result[8*(i-1) + 3] = leg_length - max;

        // HR
        dx = (body_pos[0] - d - p2[0]);
        dy = (body_pos[1] - w - p2[1]);
        dz = (body_pos[2] - p2[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        result[8*(i-1) + 4] = min - leg_length;
        result[8*(i-1) + 5] = leg_length - max;

        // HL
        dx = (body_pos[0] - d - p3[0]);
        dy = (body_pos[1] + w - p3[1]);
        dz = (body_pos[2] - p3[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        result[8*(i-1) + 6] = min - leg_length;
        result[8*(i-1) + 7] = leg_length - max;
    }
}

void WalkingForward::SaveOptimizationResult(
        const std::string& folder, const int & iter, 
        const double & opt_cost, const std::vector<double> & x, const WalkingForward* tester){
    std::string file_name = std::to_string(iter) + "_opt_result";
    saveValue(opt_cost, folder+"/"+file_name, true);
    // Raw optimization data save
    saveVector(x, folder+"/"+file_name, true);

    // time, body trajectory, foot stepping save
    BS_Basic<double, 3, 3, WalkingForward::nMiddle_pt, 2, 2> body_traj;
    tester->buildSpline(x, body_traj);

    double dt(0.005);
    double curr_time(0.);
    double pos[3];
    double vel[3];
    double acc[3];
    while(curr_time < (_tot_time+dt) ){
        body_traj.getCurvePoint(curr_time, pos);
        body_traj.getCurveDerPoint(curr_time, 1, vel);
        body_traj.getCurveDerPoint(curr_time, 2, acc);

        saveVector(pos, folder+"/"+std::to_string(iter)+"_body_pos", 3, true);
        saveVector(vel, folder+"/"+std::to_string(iter)+"_body_vel", 3, true);
        saveVector(acc, folder+"/"+std::to_string(iter)+"_body_acc", 3, true);
        saveValue(curr_time, folder+"/"+std::to_string(iter)+"_time", true);

        curr_time += dt;
    }
    
    // (num_step-1) X (3*4)
    std::vector< std::vector<double> > FootLoc;

    buildFootStepLocation(x, FootLoc, tester->_hmap);

    for(int i(0); i < (WalkingForward::_nStep -1); ++i){
        saveVector(FootLoc[i], folder+"/"+std::to_string(iter)+"_foot_loc", true);
    }
}

void WalkingForward::buildFootStepLocation(const std::vector<double> & x, 
        std::vector<std::vector<double>> & foot_loc_list, const HeightMap* hmap){
    std::vector<double> foot_loc(12); // num leg x (x, y, z)
    foot_loc_list.resize(WalkingForward::_nStep -1);

    int fr_idx, fl_idx, hr_idx, hl_idx;
    for(unsigned int i(0); i<(WalkingForward::_nStep -1); ++i){
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

void WalkingForward::InitialFinalConstraint(
        unsigned m, double * result, unsigned n, const double *x, 
        double * grad, void*data){

    (void)m;
    (void)n;
    (void)grad;
    WalkingForward* test = (WalkingForward*) data;

    int idx_offset_body = 4*test->_nStep;
    for(int i(0); i<2; ++i){
        result[i] = x[i]- test->_ini_front_foot_loc[i];
        result[i+2] = x[i+2]- test->_ini_hind_foot_loc[i];

        result[i+4] = x[i+4*(test->_nStep-2)] - test->_fin_fr_loc[i];
        result[i+6] = x[i+4*(test->_nStep-2)+2] - test->_fin_hl_loc[i];
        result[i+8] = x[i+4*(test->_nStep-1)] - test->_fin_fl_loc[i];
        result[i+10] = x[i+4*(test->_nStep-1)+2] - test->_fin_hr_loc[i];

    }
    for(int i(0); i<3; ++i){
        result[i+12] = x[i+idx_offset_body]- test->_ini_body_pos[i];
        result[i+15] = x[i+idx_offset_body + 3*(test->_nStep-1)] - test->_fin_body_pos[i];
    }
}

void WalkingForward::ProgressBodyConstraint(unsigned m, double* result, unsigned n, 
                const double *x, double *grad, void* data){

    (void) m; (void)n; (void)grad; (void)data;
    for(int i(0); i<_nStep-1; ++i){
        result[i] = x[idx_offset + 3*i] - x[idx_offset + 3*(i+1)];
    }
}

void WalkingForward::_SetInitialGuess(std::vector<double> & x){
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

    for(int i(0); i<3; ++i){
        x[i+idx_offset] = _ini_body_pos[i];
        x[i+idx_offset + 3*(_nStep-1)] = _fin_body_pos[i];
    }
    double delta_progress = (_fin_body_pos[0]-_ini_body_pos[0])/(_nStep-1);

    for(int i(0); i<_nStep; ++i){
        x[3*i + idx_offset] = _ini_body_pos[0] + delta_progress*i;
        x[3*i + idx_offset + 1] = _ini_body_pos[1];
        x[3*i + idx_offset + 2] = _ini_body_pos[2];
    }
}

void WalkingForward::nice_print_result(const std::vector<double> & x){
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
}


