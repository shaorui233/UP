#include <Cheetah3StairTop.hpp>
#include <stdio.h>
#include <utilities/save_file.hpp>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <height_map/FlatGround.hpp>
#include <ParamHandler.hpp>
#include <height_map/StairTerrain.hpp>

// TODO
//#include <height_map/TerrainFile.hpp>
#include <time.h>


Cheetah3StairTop::Cheetah3StairTop():WalkingOrientation()
{

    _ParameterSetting();

    _fr_body_to_leg_local.setZero();
    _fr_body_to_leg_local[0] = 0.25;
    _fr_body_to_leg_local[1] = -0.17;

    _fl_body_to_leg_local.setZero();
    _fl_body_to_leg_local[0] = 0.25;
    _fl_body_to_leg_local[1] = 0.17;

    _hr_body_to_leg_local.setZero();
    _hr_body_to_leg_local[0] = -0.25;
    _hr_body_to_leg_local[1] = -0.17;

    _hl_body_to_leg_local.setZero();
    _hl_body_to_leg_local[0] = -0.25;
    _hl_body_to_leg_local[1] = 0.17;


    _ini_front_foot_loc[0] = _fl_body_to_leg_local[0]; //x
    _ini_front_foot_loc[1] = _fl_body_to_leg_local[1]; //y

    _ini_hind_foot_loc[0] = _hr_body_to_leg_local[0];
    _ini_hind_foot_loc[1] = _hr_body_to_leg_local[1];

    Mat3<double> rot;
    _getRotationMatrix(_fin_body_ori[0], _fin_body_ori[1], rot);
    
    Vec3<double> fr_body_to_leg_global = rot * _fr_body_to_leg_local;
    Vec3<double> fl_body_to_leg_global = rot * _fl_body_to_leg_local;
    Vec3<double> hr_body_to_leg_global = rot * _hr_body_to_leg_local;
    Vec3<double> hl_body_to_leg_global = rot * _hl_body_to_leg_local;

    for(int i(0); i<2; ++i){
        _fin_fr_loc[i] = _fin_body_pos[i] + fr_body_to_leg_global[i];
        _fin_fl_loc[i] = _fin_body_pos[i] + fl_body_to_leg_global[i];
        _fin_hl_loc[i] = _fin_body_pos[i] + hl_body_to_leg_global[i];
        _fin_hr_loc[i] = _fin_body_pos[i] + hr_body_to_leg_global[i];
    }
    //pretty_print(rot,std::cout, "rotation");
    //printf("rotation : %f, %f\n", _fin_body_ori[0], _fin_body_ori[1]);
    //pretty_print(_hl_body_to_leg_local, std::cout, "hl local");
    //pretty_print(hl_body_to_leg_global, std::cout, "hl global");
    //pretty_print(_fin_hl_loc, "fin hl loc", 2);
}

Cheetah3StairTop::~Cheetah3StairTop(){}

double Cheetah3StairTop::ObjectiveFn(
        const std::vector<double> &x, 
        std::vector<double> & grad,
        void *d_) {

    (void)grad; 

    Cheetah3StairTop* tester = (Cheetah3StairTop*)d_;
    double cost_leg(0.);
    double cost_foot_loc(0.);
    BS_Basic<double, 3, 3, WalkingOrientation::nMiddle_pt, 2, 2> body_traj;
    BS_Basic<double, 2, 3, WalkingOrientation::nMiddle_pt, 2, 2> body_ori_traj;
    tester->buildSpline(x, body_traj, body_ori_traj);

    double des = tester->_des_leg_length; 
    //double swing_time = tester->_onestep_duration;
    HeightMap* hmap = tester->_hmap;

    //double body_pos[3]; body_pos[0] = 0.; body_pos[1] = 0.; body_pos[2] = 0.;
    //double body_ori[2]; body_ori[0] = 0.; body_ori[1] = 0.; 
    //double p0[3], p1[3], p2[3], p3[3]; //Fr, Fl, Hr, Hl
    //double dx, dy, dz;
    double leg_length;

    double curr_time(0.);
    int num_sample(500);
    double delta_t = _tot_time/(num_sample + 1.);
    double body_acc[3]; body_acc[0] = 0.; body_acc[1] = 0.; body_acc[2] = 0.;
    double body_ang_acc[2]; body_ang_acc[0] = 0.; body_ang_acc[1] = 0.; 
    double cost_der(0.);
    for(int i(0); i<num_sample; ++i){
        curr_time = i*delta_t;
        body_traj.getCurveDerPoint(curr_time, 2, body_acc);
        body_ori_traj.getCurveDerPoint(curr_time, 2, body_ang_acc);
        cost_der += 
            (body_acc[0]*body_acc[0] + body_acc[1]*body_acc[1] + body_acc[2]*body_acc[2])/num_sample;
        cost_der += 
            (body_ang_acc[0]*body_ang_acc[0] + body_ang_acc[1]*body_ang_acc[1])/num_sample;
    }

    std::vector<double> leg_length_list(4*(_nStep-1));
    std::vector<double> leg_loc_cost_list(4*(_nStep-1));
    tester->ComputeLegLength(body_traj, body_ori_traj, x, 
            leg_length_list, leg_loc_cost_list, tester);

    for(int i(1); i<tester->_nStep; ++i){
        // FR
        leg_length = leg_length_list[4*(i-1)];
        cost_leg += ( (leg_length - des) * (leg_length -des) );
        cost_foot_loc += leg_loc_cost_list[4*(i-1)];

        // FL
        leg_length = leg_length_list[4*(i-1) + 1];
        cost_leg += ( (leg_length - des) * (leg_length -des) );
        cost_foot_loc += leg_loc_cost_list[4*(i-1) + 1];

        // HR
        leg_length = leg_length_list[4*(i-1) + 2];
        cost_leg += (leg_length - des) * (leg_length -des);
        cost_foot_loc += leg_loc_cost_list[4*(i-1) + 2];

        /* HL */
        leg_length = leg_length_list[4*(i-1) + 3];
        cost_leg += (leg_length - des) * (leg_length -des);
        cost_foot_loc += leg_loc_cost_list[4*(i-1) + 3];
    }

    double cost_sum = (cost_leg*50.+cost_der + cost_foot_loc*200.);
    //double cost_sum = (cost_leg*200.+cost_der/10.);
    //double cost_sum = (0.3*cost_leg + cost_foot_loc);
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
        //hmap->SaveHeightMap(tester->_folder_name);
        // TEST
        hmap->SaveHeightMap("2019-03-05-00_00_03");
    }

    if( (  (tester->_count)%(tester->_intermittent_step_size) ) == 0){
        // Save a current one
        SaveOptimizationResult(tester->_folder_name, tester->_count, 
                cost_sum, 
                x, tester);
        // Print Result
        printf("%dth iter, curr cost:(%f, %f, %f), sum: %f \n", 
                tester->_count, cost_leg, cost_foot_loc, cost_der, cost_sum);
        nice_print_result(x);
        printf("\n");
    }
    return cost_sum;
}

bool Cheetah3StairTop::SolveOptimization(){
    // Reset Problem
    _intermittent_step_size = 500;
    _count = 0;
    if(_hmap) delete _hmap;
    //_hmap = new FlatGround(); 
    _hmap = new StairTerrain(_num_stair, _x_start, _y_start, _width, _height, _depth); 
    // End of Reset 
    int num_opt_var = 4*_nStep + 3*_nStep + 2*_nStep;
    nlopt::opt* test = new nlopt::opt(nlopt::LN_COBYLA, num_opt_var);
    //nlopt::opt* test = new nlopt::opt(nlopt::LN_AUGLAG, num_opt_var);
    //nlopt::opt* test = new nlopt::opt(nlopt::GN_ISRES, num_opt_var);
    //nlopt::opt* test = new nlopt::opt(nlopt::LN_AUGLAG, num_opt_var);

    test->set_min_objective(Cheetah3StairTop::ObjectiveFn, this);

    std::vector<double> lb(num_opt_var);
    std::vector<double> ub(num_opt_var);

    for(int i(0);i<_nStep; ++i){
        // x (foot)
        lb[4*i] = -0.3;
        lb[4*i + 2] = -0.3;
        ub[4*i] = 3.0;
        ub[4*i + 2] = 3.0;

        // y (foot)
        lb[4*i + 1] = -2.2;
        lb[4*i + 3] = -2.2;
        ub[4*i + 1] = 0.8;
        ub[4*i + 3] = 0.8;
    }
    int idx_offset = 4*_nStep;
    for(int i(0); i<_nStep; ++i){
        // lower
        lb[idx_offset + 3*i] = 0.; // x
        lb[idx_offset + 3*i + 1] = -1.9; // y
        lb[idx_offset + 3*i + 2] = 0.2; // z

        ub[idx_offset + 3*i] = 2.7; // x
        ub[idx_offset + 3*i + 1] = 0.6; // y
        ub[idx_offset + 3*i + 2] = 1.4; // z
    }
    idx_offset = 4*_nStep + 3*_nStep;
    for(int i(0); i<_nStep; ++i){
        // lower
        lb[idx_offset + 2*i] = -1.0; // pitch
        lb[idx_offset + 2*i + 1] = -M_PI; // yaw

        ub[idx_offset + 2*i] = 1.0; // pitch
        ub[idx_offset + 2*i + 1] = M_PI; // yaw
    }
    test->set_lower_bounds(lb);
    test->set_upper_bounds(ub);

    int num_kin_inequality = 8 * (_nStep - 1);
    std::vector<double> tol_ieq_kin(num_kin_inequality);
    for(int i(0); i<num_kin_inequality; ++i){ tol_ieq_kin[i] = 1e-8; }
    test->add_inequality_mconstraint(WalkingOrientation::KinematicsConstraint, this, tol_ieq_kin);

    int num_ini_final = 2*6 + 3*2 + 2*2;
    std::vector<double> tol_eq(num_ini_final);
    for(int i(0); i<num_ini_final; ++i){ tol_eq[i] = 1e-8; }
    test->add_equality_mconstraint(WalkingOrientation::InitialFinalConstraint, this, tol_eq);


    test->set_xtol_rel(1e-9);
    test->set_ftol_rel(1e-10);
    test->set_maxeval(_max_eval); 
    test->set_maxtime(_max_time);

    std::vector<double> x(num_opt_var);
    _SetInitialGuess(x);
    double opt_f;

    pretty_print(x, "initial guess");
    nlopt::result opt_result = test->optimize(x, opt_f);
    printf("result, cost: %d, %f\n", opt_result, opt_f);
    saveVector(x, "opt_result_global");
    pretty_print(x, "opt_result_global");
    //SaveOptimizationResult(_folder_name, 1, opt_f, x, this);
    // TEST
    SaveOptimizationResult("2019-03-05-00_00_03", 1, opt_f, x, this);
    nice_print_result(x);
    printf("\n");

    delete test;

    return true;
}



void Cheetah3StairTop::_ParameterSetting(){
    ParamHandler handler(THIS_COM"optimization/config/optimization_setup.yaml");
    std::vector<double> vec_tmp;
    handler.getVector<double>("robot_initial_posture", vec_tmp);
    _ini_body_pos[0] = vec_tmp[0];
    _ini_body_pos[1] = vec_tmp[1];
    _ini_body_pos[2] = vec_tmp[2];

    _ini_body_ori[0] = vec_tmp[3];
    _ini_body_ori[1] = vec_tmp[4];

    handler.getVector<double>("robot_final_posture", vec_tmp);
    _fin_body_pos[0] = vec_tmp[0];
    _fin_body_pos[1] = vec_tmp[1];
    _fin_body_pos[2] = vec_tmp[2];

    _fin_body_ori[0] = vec_tmp[3];
    _fin_body_ori[1] = vec_tmp[4];


    handler.getValue<double>("min_leg_length", _min_leg_length);
    handler.getValue<double>("max_leg_length", _max_leg_length);
    handler.getValue<double>("des_leg_length", _des_leg_length);

    handler.getValue<int>("max_eval", _max_eval);
    handler.getValue<int>("max_time", _max_time);
    
    // Stair Terrain
    handler.getValue<int>("stair_terrain", "num_stair", _num_stair);
    handler.getValue<double>("stair_terrain", "x_start", _x_start);
    handler.getValue<double>("stair_terrain", "y_start", _y_start);

    handler.getValue<double>("stair_terrain", "width", _width);
    handler.getValue<double>("stair_terrain", "depth", _depth);
    handler.getValue<double>("stair_terrain", "height", _height);
}
