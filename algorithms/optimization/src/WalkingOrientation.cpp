#include <WalkingOrientation.hpp>
#include <stdio.h>
#include <utilities/save_file.hpp>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <height_map/SeparateTerrain.hpp>
#include <height_map/FlatGround.hpp>
#include <height_map/RandomTerrain.hpp>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <Math/orientation_tools.h>

double ** middle_pt_pos = new double*[WalkingOrientation::nMiddle_pt];
double ** middle_pt_ori = new double*[WalkingOrientation::nMiddle_pt];

WalkingOrientation::WalkingOrientation(){ 
    _intermittent_step_size = 50000;
    _hmap = NULL;
    for(int i(0); i<nMiddle_pt; ++i) middle_pt_pos[i] = new double[3];
    for(int i(0); i<nMiddle_pt; ++i) middle_pt_ori[i] = new double[2];
}

WalkingOrientation::~WalkingOrientation(){
    for(int i(0); i<nMiddle_pt; ++i){
        delete [] middle_pt_pos[i];
        delete [] middle_pt_ori[i];
    }
    delete [] middle_pt_pos;
    delete [] middle_pt_ori;
}
void WalkingOrientation::ComputeLegLength(
        BS_Basic<double, 3, 3, nMiddle_pt, 2, 2> & pos_trj,
        BS_Basic<double, 2, 3, nMiddle_pt, 2, 2> & ori_trj,
        const std::vector<double> & x, 
        std::vector<double> & leg_length_list, 
        std::vector<double> & leg_loc_cost_list, 
        WalkingOrientation* tester){

    double swing_time = tester->_onestep_duration;
    HeightMap* hmap = tester->_hmap;

    double body_pos[3]; body_pos[0] = 0.; body_pos[1] = 0.; body_pos[2] = 0.;
    double body_ori[2]; body_ori[0] = 0.; body_ori[1] = 0.; 
    double p0[3], p1[3], p2[3], p3[3]; //Fr, Fl, Hr, Hl
    double dx, dy, dz;

    double curr_time(0.);

    for(int i(1); i<tester->_nStep; ++i){
        curr_time = i *swing_time;
        pos_trj.getCurvePoint(curr_time, body_pos);
        ori_trj.getCurvePoint(curr_time, body_ori);

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
        Mat3<double> rot;
        tester->_getRotationMatrix(body_ori[0], body_ori[1], rot);
        Vec3<double> fr_body_to_leg_global = rot * tester->_fr_body_to_leg_local;
        Vec3<double> fl_body_to_leg_global = rot * tester->_fl_body_to_leg_local;
        Vec3<double> hr_body_to_leg_global = rot * tester->_hr_body_to_leg_local;
        Vec3<double> hl_body_to_leg_global = rot * tester->_hl_body_to_leg_local;

        // FR
        dx = (body_pos[0] + fr_body_to_leg_global[0] - p0[0]);
        dy = (body_pos[1] + fr_body_to_leg_global[1] - p0[1]);
        dz = (body_pos[2] + fr_body_to_leg_global[2] - p0[2]);
        leg_length_list[4*(i-1)] = sqrt(dx*dx + dy*dy + dz*dz);
        leg_loc_cost_list[4*(i-1)] = dx*dx + dy*dy;

        // FL
        dx = (body_pos[0] + fl_body_to_leg_global[0] - p1[0]);
        dy = (body_pos[1] + fl_body_to_leg_global[1] - p1[1]);
        dz = (body_pos[2] + fl_body_to_leg_global[2] - p1[2]);
        leg_length_list[4*(i-1) + 1] = sqrt(dx*dx + dy*dy + dz*dz);
        leg_loc_cost_list[4*(i-1) + 1] = dx*dx + dy*dy;

        // HR
        dx = (body_pos[0] + hr_body_to_leg_global[0] - p2[0]);
        dy = (body_pos[1] + hr_body_to_leg_global[1] - p2[1]);
        dz = (body_pos[2] + hr_body_to_leg_global[2] - p2[2]);
        leg_length_list[4*(i-1) + 2] = sqrt(dx*dx + dy*dy + dz*dz);
        leg_loc_cost_list[4*(i-1) + 2] = dx*dx + dy*dy;

        /* HL */
        dx = (body_pos[0] + hl_body_to_leg_global[0] - p3[0]);
        dy = (body_pos[1] + hl_body_to_leg_global[1] - p3[1]);
        dz = (body_pos[2] + hl_body_to_leg_global[2] - p3[2]);
        leg_length_list[4*(i-1) + 3] = sqrt(dx*dx + dy*dy + dz*dz);
        leg_loc_cost_list[4*(i-1) + 3] = dx*dx + dy*dy;
    }
}
 
void WalkingOrientation::buildSpline(
        const std::vector<double> & x, 
        BS_Basic<double, 3, 3, nMiddle_pt, 2, 2> & pos_spline,
        BS_Basic<double, 2, 3, nMiddle_pt, 2, 2> & ori_spline){

    double init[9];
    double fin[9];

    int num_mid_pt = WalkingOrientation::_nStep-2;
    int degree = 1;

    // Position
    int idx_offset = 4*WalkingOrientation::_nStep;
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
            middle_pt_pos[degree*i+j][0] = x[idx_offset + 3*(i+1)];
            middle_pt_pos[degree*i+j][1] = x[idx_offset + 3*(i+1)+1];
            middle_pt_pos[degree*i+j][2] = x[idx_offset + 3*(i+1)+2];
        }
    }
    pos_spline.SetParam(init, fin, middle_pt_pos, _tot_time);

    // Orientation
    double init_ori[6];
    double fin_ori[6];
    idx_offset = 4*WalkingOrientation::_nStep + 3*WalkingOrientation::_nStep;
    init_ori[0] = x[0 + idx_offset]; // starting loc
    init_ori[1] = x[1 + idx_offset]; 

    init_ori[2] = 0.; //velocity
    init_ori[3] = 0.; 

    init_ori[4] = 0.; //acceleration 
    init_ori[5] = 0.; 


    fin_ori[0] = x[0 + idx_offset + 2*(num_mid_pt+1)]; // end loc
    fin_ori[1] = x[1 + idx_offset + 2*(num_mid_pt+1)];

    fin_ori[2] = 0.;
    fin_ori[3] = 0.;

    fin_ori[4] = 0.;
    fin_ori[5] = 0.;

    // middle point allocation
    for(int i(0); i<num_mid_pt; ++i){
        for(int j(0); j<degree; ++j){
            middle_pt_ori[degree*i+j][0] = x[idx_offset + 2*(i+1)];
            middle_pt_ori[degree*i+j][1] = x[idx_offset + 2*(i+1)+1];
        }
    }
    ori_spline.SetParam(init_ori, fin_ori, middle_pt_ori, _tot_time);
}

// TODO
void WalkingOrientation::KinematicsConstraint(unsigned m, double* result, unsigned n, 
        const double *x, double *grad, void* data){

    (void)m;
    (void)grad;

    WalkingOrientation* tester = (WalkingOrientation*) data;
    double d = tester->_half_body_length;
    double w = tester->_half_body_width;
    double swing_time = tester->_onestep_duration;
    HeightMap* hmap = tester->_hmap;
    BS_Basic<double, 3, 3, WalkingOrientation::nMiddle_pt, 2, 2> body_traj;
    BS_Basic<double, 2, 3, WalkingOrientation::nMiddle_pt, 2, 2> body_ori_traj;


    std::vector<double> x_vec(n);
    for(unsigned int i(0); i<n; ++i){ x_vec[i] = x[i]; }
    tester->buildSpline(x_vec, body_traj, body_ori_traj);
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

void WalkingOrientation::SaveOptimizationResult(
        const std::string& folder, const int & iter, 
        const double & opt_cost, const std::vector<double> & x, const WalkingOrientation* tester){

    std::string file_name = std::to_string(iter) + "_opt_result";
    saveValue(opt_cost, folder+"/"+file_name, true);
    // Raw optimization data save
    saveVector(x, folder+"/"+file_name, true);

    // time, body trajectory, foot stepping save
    BS_Basic<double, 3, 3, WalkingOrientation::nMiddle_pt, 2, 2> body_traj;
    BS_Basic<double, 2, 3, WalkingOrientation::nMiddle_pt, 2, 2> ori_traj;
    tester->buildSpline(x, body_traj, ori_traj);

    double dt(0.005);
    double curr_time(0.);
    double pos[3];
    double vel[3];
    double acc[3];

    double ori[2];
    double ang_vel[2];
    double ang_acc[2];
    while(curr_time < (_tot_time+dt) ){
        body_traj.getCurvePoint(curr_time, pos);
        body_traj.getCurveDerPoint(curr_time, 1, vel);
        body_traj.getCurveDerPoint(curr_time, 2, acc);

        saveVector(pos, folder+"/"+std::to_string(iter)+"_body_pos", 3, true);
        saveVector(vel, folder+"/"+std::to_string(iter)+"_body_vel", 3, true);
        saveVector(acc, folder+"/"+std::to_string(iter)+"_body_acc", 3, true);


        ori_traj.getCurvePoint(curr_time, ori);
        ori_traj.getCurveDerPoint(curr_time, 1, ang_vel);
        ori_traj.getCurveDerPoint(curr_time, 2, ang_acc);

        saveVector(ori, folder+"/"+std::to_string(iter)+"_body_ori", 2, true);
        saveVector(ang_vel, folder+"/"+std::to_string(iter)+"_body_ang_vel", 2, true);
        saveVector(ang_acc, folder+"/"+std::to_string(iter)+"_body_ang_acc", 2, true);


        saveValue(curr_time, folder+"/"+std::to_string(iter)+"_time", true);

        curr_time += dt;
    }
    
    // (num_step-1) X (3*4)
    std::vector< std::vector<double> > FootLoc;

    buildFootStepLocation(x, FootLoc, tester->_hmap);

    for(int i(0); i < (WalkingOrientation::_nStep -1); ++i){
        saveVector(FootLoc[i], folder+"/"+std::to_string(iter)+"_foot_loc", true);
    }
}

void WalkingOrientation::buildFootStepLocation(const std::vector<double> & x, 
        std::vector<std::vector<double>> & foot_loc_list, const HeightMap* hmap){

    std::vector<double> foot_loc(12); // num leg x (x, y, z)
    foot_loc_list.resize(WalkingOrientation::_nStep -1);

    int fr_idx, fl_idx, hr_idx, hl_idx;
    for(unsigned int i(0); i<(WalkingOrientation::_nStep -1); ++i){
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

void WalkingOrientation::InitialFinalConstraint(
        unsigned m, double * result, unsigned n, const double *x, 
        double * grad, void*data){

    (void)m;
    (void)n;
    (void)grad;
    WalkingOrientation* test = (WalkingOrientation*) data;

    int idx_offset_body = 4*test->_nStep;
    for(int i(0); i<2; ++i){
        result[i] = x[i]- test->_ini_front_foot_loc[i];
        result[i+2] = x[i+2]- test->_ini_hind_foot_loc[i];

        result[i+4] = x[i+4*(test->_nStep-2)] - test->_fin_fr_loc[i];
        result[i+6] = x[i+4*(test->_nStep-2)+2] - test->_fin_hl_loc[i];
        result[i+8] = x[i+4*(test->_nStep-1)] - test->_fin_fl_loc[i];
        result[i+10] = x[i+4*(test->_nStep-1)+2] - test->_fin_hr_loc[i];

    }
    // Body Position Initial & Final
    for(int i(0); i<3; ++i){
        result[i+12] = x[i+idx_offset_body]- test->_ini_body_pos[i];
        result[i+15] = x[i+idx_offset_body + 3*(test->_nStep-1)] - test->_fin_body_pos[i];
    }
    // Body Orientation Initial & Final (pitch & yaw)
    int idx_offset_ori = 4*test->_nStep + 3*test->_nStep;
    for(int i(0); i<2; ++i){
        result[i+18] = x[i+idx_offset_ori]- test->_ini_body_ori[i];
        result[i+20] = x[i+idx_offset_ori + 2*(test->_nStep-1)] - test->_fin_body_ori[i];
     }
}

void WalkingOrientation::_SetInitialGuess(std::vector<double> & x){
    // Foot initial and Final
    for(int i(0); i<2; ++i){
        x[i] = _ini_front_foot_loc[i];
        x[i+2] =  _ini_hind_foot_loc[i];
        x[i+4*(_nStep-2)] = _fin_fr_loc[i];
        x[i+4*(_nStep-2)+2] = _fin_hl_loc[i];
        x[i+4*(_nStep-1)] = _fin_fl_loc[i];
        x[i+4*(_nStep-1)+2] = _fin_hr_loc[i];
    }

    // TODO
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

    // TODO
    double delta_progress = (_fin_body_pos[0]-_ini_body_pos[0])/(_nStep-1);

    for(int i(0); i<_nStep; ++i){
        x[3*i + idx_offset] = _ini_body_pos[0] + delta_progress*i;
        x[3*i + idx_offset + 1] = _ini_body_pos[1];
        x[3*i + idx_offset + 2] = _ini_body_pos[2];
    }
}

void WalkingOrientation::nice_print_result(const std::vector<double> & x){
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
        printf("%d th body ori: (%f, %f)\n", i,
                x[2*i + idx_offset], x[2*i + idx_offset +1]);
    }
}


void WalkingOrientation::_getRotationMatrix(const double & pitch, 
        const double & yaw, Mat3<double> & rot){

    Mat3<double> Ry, Rz;

    double sy = std::sin(pitch);
    double cy = std::cos(pitch);
    double sz = std::sin(yaw);
    double cz = std::cos(yaw);
    Ry << cy, 0, sy,
       0, 1, 0,
       -sy, 0, cy;
    Rz << cz, -sz, 0,
       sz, cz, 0,
       0, 0, 1;

    rot = Rz * Ry;
}
