#include <BodyAccMin.hpp>
#include <stdio.h>
#include <utilities/save_file.hpp>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <height_map/SeparateTerrain.hpp>
#include <height_map/FlatGround.hpp>
#include <height_map/RandomTerrain.hpp>
#include <time.h>

BodyAccMin::BodyAccMin():WalkingForward()
{
    _min_leg_length = 0.21;
    _max_leg_length = 0.31;

    _des_leg_length = 0.25;

    _ini_front_foot_loc[0] = 0.4;
    _ini_front_foot_loc[1] = 0.42;

    _ini_hind_foot_loc[0] = 0.0;
    _ini_hind_foot_loc[1] = 0.18;

    _fin_fr_loc[0] = 1.6;
    _fin_fr_loc[1] = 0.18;

    _fin_hl_loc[0] = 1.2;
    _fin_hl_loc[1] = 0.42;

    _fin_fl_loc[0] = 1.6;
    _fin_fl_loc[1] = 0.42;

    _fin_hr_loc[0] = 1.2;
    _fin_hr_loc[1] = 0.18;

    _ini_body_pos[0] = 0.2;
    _ini_body_pos[1] = 0.3;
    _ini_body_pos[2] = 0.25;

    _fin_body_pos[0] = 1.4;
    _fin_body_pos[1] = 0.3;
    _fin_body_pos[2] = 0.25;
}

BodyAccMin::~BodyAccMin(){}

double BodyAccMin::ObjectiveFn(
        const std::vector<double> &x, 
        std::vector<double> & grad,
        void *d_) {

    (void)grad; 

    BodyAccMin* tester = (BodyAccMin*)d_;
    double cost_leg(0.);
    BS_Basic<double, 3, 3, WalkingForward::nMiddle_pt, 2, 2> body_traj;
    tester->buildSpline(x, body_traj);

    double des = tester->_des_leg_length; 
    double d = tester->_half_body_length;
    double w = tester->_half_body_width;
    double swing_time = tester->_onestep_duration;
    HeightMap* hmap = tester->_hmap;

    double body_pos[3]; body_pos[0] = 0.; body_pos[1] = 0.; body_pos[2] = 0.;
    double p0[3], p1[3], p2[3], p3[3]; //Fr, Fl, Hr, Hl
    double dx, dy, dz;
    double leg_length;

    double curr_time(0.);
    int num_sample(500);
    double delta_t = _tot_time/(num_sample + 1.);
    double body_acc[3]; body_acc[0] = 0.; body_acc[1] = 0.; body_acc[2] = 0.;
    double cost_der(0.);
    for(int i(0); i<num_sample; ++i){
        curr_time = i*delta_t;
        body_traj.getCurveDerPoint(curr_time, 2, body_acc);
        cost_der += 
            (body_acc[0]*body_acc[0] + body_acc[1]*body_acc[1] + body_acc[2]*body_acc[2])/num_sample;
    }
    for(int i(1); i<tester->_nStep; ++i){
        curr_time = i *swing_time;
        body_traj.getCurvePoint(curr_time, body_pos);

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

        cost_leg += ( (leg_length - des) * (leg_length -des) );

        // FL
        dx = (body_pos[0] + d - p1[0]);
        dy = (body_pos[1] + w - p1[1]);
        dz = (body_pos[2] - p1[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        cost_leg += ( (leg_length - des) * (leg_length -des) );

        // HR
        dx = (body_pos[0] - d - p2[0]);
        dy = (body_pos[1] - w - p2[1]);
        dz = (body_pos[2] - p2[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        cost_leg += (leg_length - des) * (leg_length -des);

        /* HL */
        dx = (body_pos[0] - d - p3[0]);
        dy = (body_pos[1] + w - p3[1]);
        dz = (body_pos[2] - p3[2]);
        leg_length = sqrt(dx*dx + dy*dy + dz*dz);

        cost_leg += (leg_length - des) * (leg_length -des);
    }

    double cost_sum = (cost_leg*200.+cost_der/10.);
    //double cost_sum = (cost_leg);

    tester->_count++;
    if(tester->_count == 1){  // In the first loop, save height map
        char folder_name[80];
        time_t  now = time(0);
        struct tm  tstruct;
        tstruct = *localtime(&now);
        strftime(folder_name, sizeof(folder_name), "%Y-%m-%d-%H_%M_%S", &tstruct);
        tester->_folder_name = folder_name;
        create_folder(tester->_folder_name);
        hmap->SaveHeightMap(tester->_folder_name);
    }

    if( (  (tester->_count)%(tester->_intermittent_step_size) ) == 0){
        // Save a current one
        SaveOptimizationResult(tester->_folder_name, tester->_count, 
                cost_sum, 
                x, tester);
        // Print Result
        printf("%dth iter, curr cost:(%f, %f), sum: %f \n", 
                tester->_count, cost_leg, cost_der, cost_sum);
        nice_print_result(x);
        printf("\n");
    }
    return cost_sum;
}

bool BodyAccMin::SolveOptimization(){
    // Reset Problem
    _intermittent_step_size = 50000;
    _count = 0;
    if(_hmap) delete _hmap;
    _hmap = new RandomTerrain(13, 0.45, 1.15, 0.0, 0.6);
    // End of Reset 
    int num_opt_var = 4*_nStep + 3*_nStep;
    //nlopt::opt* test = new nlopt::opt(nlopt::LN_COBYLA, num_opt_var);
    //nlopt::opt* test = new nlopt::opt(nlopt::LN_AUGLAG, num_opt_var);
    nlopt::opt* test = new nlopt::opt(nlopt::GN_ISRES, num_opt_var);
    //nlopt::opt* test = new nlopt::opt(nlopt::LN_AUGLAG, num_opt_var);

    test->set_min_objective(BodyAccMin::ObjectiveFn, this);

    std::vector<double> lb(4*_nStep + 3*_nStep);
    std::vector<double> ub(4*_nStep + 3*_nStep);

    for(int i(0);i<_nStep; ++i){
        // x (foot)
        lb[4*i] = 0.;
        lb[4*i + 2] = 0.;
        ub[4*i] = 1.6;
        ub[4*i + 2] = 1.6;

        // y (foot)
        lb[4*i + 1] = 0.;
        lb[4*i + 3] = 0.;
        ub[4*i + 1] = 0.6;
        ub[4*i + 3] = 0.6;
    }
    int idx_offset = 4*_nStep;
    for(int i(0); i<_nStep; ++i){
        // lower
        lb[idx_offset + 3*i] = 0.; // x
        lb[idx_offset + 3*i + 1] = 0.; // y
        lb[idx_offset + 3*i + 2] = 0.1; // z

        ub[idx_offset + 3*i] = 1.6; // x
        ub[idx_offset + 3*i + 1] = 0.6; // y
        ub[idx_offset + 3*i + 2] = 0.4; // z
    }
    test->set_lower_bounds(lb);
    test->set_upper_bounds(ub);

    int num_kin_inequality = 8 * (_nStep - 1);
    std::vector<double> tol_ieq_kin(num_kin_inequality);
    for(int i(0); i<num_kin_inequality; ++i){ tol_ieq_kin[i] = 1e-8; }
    test->add_inequality_mconstraint(WalkingForward::KinematicsConstraint, this, tol_ieq_kin);

    int body_progress = _nStep-1;
    std::vector<double> tol_ieq_progress(body_progress);
    for(int i(0); i<body_progress; ++i){ tol_ieq_progress[i] = 1e-8; }
    test->add_inequality_mconstraint(WalkingForward::ProgressBodyConstraint, this, tol_ieq_progress);


    int num_ini_final = 2*6 + 3*2;
    std::vector<double> tol_eq(num_ini_final);
    for(int i(0); i<num_ini_final; ++i){ tol_eq[i] = 1e-8; }
    test->add_equality_mconstraint(WalkingForward::InitialFinalConstraint, this, tol_eq);


    test->set_xtol_rel(1e-9);
    test->set_ftol_rel(1e-10);
    test->set_maxeval(100000);

    std::vector<double> x(num_opt_var);
    _SetInitialGuess(x);
    double opt_f;

    pretty_print(x, "initial guess");
    nlopt::result opt_result = test->optimize(x, opt_f);
    printf("result, cost: %d, %f\n", opt_result, opt_f);
    saveVector(x, "opt_result_global");
    pretty_print(x, "opt_result_global");
    SaveOptimizationResult(_folder_name, 1, opt_f, x, this);
    printf("\n");

    delete test;
    test = NULL;

    // Local Search again 
    _intermittent_step_size = 1000;
    test = new nlopt::opt(nlopt::LN_COBYLA, num_opt_var);
    test->set_min_objective(BodyAccMin::ObjectiveFn, this);

    test->set_lower_bounds(lb);
    test->set_upper_bounds(ub);
    test->add_inequality_mconstraint(WalkingForward::KinematicsConstraint, this, tol_ieq_kin);
    test->add_inequality_mconstraint(WalkingForward::ProgressBodyConstraint, this, tol_ieq_progress);
    test->add_equality_mconstraint(WalkingForward::InitialFinalConstraint, this, tol_eq);
    test->set_xtol_rel(1e-9);
    test->set_ftol_rel(1e-10);
    test->set_maxeval(10000);
    test->set_maxtime(600);

    opt_result = test->optimize(x, opt_f);
    printf("result, cost: %d, %f\n", opt_result, opt_f);


    SaveOptimizationResult(_folder_name, 0, opt_f, x, this);
    saveVector(x, "opt_result");
    nice_print_result(x);

    delete test;

    return true;
}

