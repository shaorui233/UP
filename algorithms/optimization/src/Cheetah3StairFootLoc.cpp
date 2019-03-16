#include <Cheetah3StairFootLoc.hpp>
#include <stdio.h>
#include <utilities/save_file.hpp>
#include <Utilities/Utilities_print.h>
#include <cppTypes.h>
#include <nlopt.hpp>
#include <height_map/FlatGround.hpp>
#include <ParamHandler.hpp>
#include <height_map/StairTerrain.hpp>
#include <height_map/BoxTerrain.hpp>
#include <time.h>

Cheetah3StairFootLoc::Cheetah3StairFootLoc():WalkingFootLoc()
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
    _getRotationMatrix(_fin_body_ori, rot);
    
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
    //printf("rotation : %f\n", _fin_body_ori);
    //pretty_print(_hl_body_to_leg_local, std::cout, "hl local");
    //pretty_print(hl_body_to_leg_global, std::cout, "hl global");
    //pretty_print(_fin_hl_loc, "fin hl loc", 2);
}

Cheetah3StairFootLoc::~Cheetah3StairFootLoc(){}

double Cheetah3StairFootLoc::ObjectiveFn(
        const std::vector<double> &x, 
        std::vector<double> & grad,
        void *d_) {

    (void)grad; 

    Cheetah3StairFootLoc* tester = (Cheetah3StairFootLoc*)d_;
    HeightMap* hmap = tester->_hmap;
    double cost_sum(0.);
    for(size_t i(0); i< x.size(); ++i){
        cost_sum += (x[i] - tester->_x_initial[i])* (x[i] - tester->_x_initial[i]);
    }

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
        tester->_update_full_variable(x);
        SaveOptimizationResult(tester->_folder_name, tester->_count, 
                cost_sum, 
                tester->_full_x, tester);
        // Print Result
        printf("%dth iter, curr cost: %f \n", tester->_count, cost_sum);
        nice_print_result(tester->_full_x);
        printf("\n");
    }
    return cost_sum;
}

void Cheetah3StairFootLoc::_update_full_variable(const std::vector<double> & x){
    for(size_t i(0);i<x.size(); ++i) _full_x[i] = x[i];
}
double Cheetah3StairFootLoc::ObjectiveFn_LandingMargin(
        const std::vector<double> &x, 
        std::vector<double> & grad,
        void *d_) {

    (void)grad; 

    Cheetah3StairFootLoc* tester = (Cheetah3StairFootLoc*) d_;
    HeightMap* hmap = tester->_hmap;
    double cost_sum(0.);

    double x_loc, y_loc, x_check, y_check;
    double landing_height;
    double height_var_sum(0.);

    double res(0.002);
    double check_region = tester->_landing_loc_region;
    int num_check = floor(check_region/res);
    for(int i(0); i<2*tester->_nStep; ++i){
        x_loc = x[2*i];
        y_loc = x[2*i+1];

        height_var_sum = 0.;
        landing_height = hmap->getHeight(x_loc, y_loc);

        for(int x_inc(0); x_inc<num_check; ++x_inc){
            for(int y_inc(0); y_inc<num_check; ++y_inc){
                x_check = x_loc + res*x_inc - check_region/2.;
                y_check = y_loc + res*y_inc - check_region/2.;
                height_var_sum += fabs(hmap->getHeight(x_check, y_check) - landing_height);
            }
        }
        height_var_sum /=num_check;
        height_var_sum /=num_check;

        cost_sum += height_var_sum;
        //printf("num check:%d\n", num_check);
        //printf("x, y loc: %f, %f\n", x_loc, y_loc);
        //printf("landing height: %f, var sum: %f\n", landing_height, height_var_sum);
        //printf("result: %f\n", result[i]);
    }

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
        tester->_update_full_variable(x);
        SaveOptimizationResult(tester->_folder_name, tester->_count, 
                cost_sum, 
                tester->_full_x, tester);
        // Print Result
        printf("%dth iter, curr cost: %f \n", tester->_count, cost_sum);
        nice_print_result(tester->_full_x);
        printf("\n");
    }
    return cost_sum;
}


void Cheetah3StairFootLoc::_SetInitialPitch(std::vector<double> & x){
    std::vector< std::vector<double> > FootLoc;
    buildFootStepLocation(x, FootLoc, _hmap);
   double forward_x, forward_z;
   double hind_x, hind_z;

    for(int i(1); i<_nStep-1; ++i){
        forward_x = (FootLoc[i][0] + FootLoc[i][3])/2.;
        forward_z = (FootLoc[i][2] + FootLoc[i][5])/2.;

        hind_x = (FootLoc[i][6] + FootLoc[i][9])/2.;
        hind_z = (FootLoc[i][8] + FootLoc[i][11])/2.;
       
        //x[idx_offset + 3*_nStep + i] = 
          //-std::atan((x[3*i + idx_offset + 2] - x[3*(i-1) + idx_offset+2])/delta_progress);
        x[idx_offset + 3*_nStep + i] = -std::atan((forward_z-hind_z)/(forward_x - hind_x));
    }
}
void Cheetah3StairFootLoc::StickToInitialConstraint(
        unsigned m, double * result, unsigned n, const double *x, 
        double * grad, void*data){

    (void)m;
    (void)grad;
    Cheetah3StairFootLoc* tester = (Cheetah3StairFootLoc*)data;
    double error_bound(0.1);
    double error_from_initial(0.0);
    for(unsigned int i(0); i<n; ++i){
        error_from_initial = sqrt(  (x[i] - tester->_x_initial[i])*(x[i] - tester->_x_initial[i]) );
        result[i] = error_from_initial - error_bound;
    }
}


bool Cheetah3StairFootLoc::SolveOptimization(){
    // Reset Problem
    _intermittent_step_size = 500;
    _count = 0;
    if(_hmap) delete _hmap;
    _hmap = new BoxTerrain(_x_start, _y_start, _width, _height, _depth); 
    //_hmap = new StairTerrain(_num_stair, _x_start, _y_start, _width, _height, _depth); 
    // End of Reset 
    nlopt::opt* test = new nlopt::opt(nlopt::LN_COBYLA, num_opt_var);

    test->set_min_objective(Cheetah3StairFootLoc::ObjectiveFn, this);
    //test->set_min_objective(Cheetah3StairFootLoc::ObjectiveFn_LandingMargin, this);

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
    test->set_lower_bounds(lb);
    test->set_upper_bounds(ub);

    // Landing location margin
    int num_landing_margin_inequality = 2 * (_nStep);
    std::vector<double> tol_ieq_land(num_landing_margin_inequality);
    for(int i(0); i<num_landing_margin_inequality; ++i){ tol_ieq_land[i] = 1e-8; }
    test->add_inequality_mconstraint(WalkingFootLoc::LandingMarginConstraint, this, tol_ieq_land);

    int num_initial_bound_inequality = 2 * (_nStep);
    std::vector<double> tol_ieq_ini(num_initial_bound_inequality);
    for(int i(0); i<num_initial_bound_inequality; ++i){ tol_ieq_ini[i] = 1e-8; }
    test->add_inequality_mconstraint(Cheetah3StairFootLoc::StickToInitialConstraint, this, tol_ieq_ini);


    // Initial and Final constraints
    int num_ini_final = 2*6;
    std::vector<double> tol_eq(num_ini_final);
    for(int i(0); i<num_ini_final; ++i){ tol_eq[i] = 1e-8; }
    test->add_equality_mconstraint(WalkingFootLoc::InitialFinalConstraint, this, tol_eq);

    test->set_xtol_rel(1e-13);
    test->set_ftol_rel(1e-13);
    test->set_maxeval(_max_eval); 
    test->set_maxtime(_max_time);

    std::vector<double> x(num_opt_var);
    _full_x.resize(num_full_var);
    _SetInitialGuess(_full_x);
    _SetInitialPitch(_full_x);
    for(int i(0);i<num_opt_var; ++i){ x[i] = _full_x[i]; }
    _x_initial = x;
    double opt_f;

    pretty_print(x, "initial guess");
    nlopt::result opt_result = test->optimize(x, opt_f);
    printf("result, cost: %d, %f\n", opt_result, opt_f);
    _update_full_variable(x);
    //SaveOptimizationResult(_folder_name, 1, opt_f, x, this);
    // TEST
    SaveOptimizationResult("2019-03-05-00_00_03", 1, opt_f, _full_x, this);
    nice_print_result(_full_x);
    printf("\n");

    delete test;

    return true;
}



void Cheetah3StairFootLoc::_ParameterSetting(){
    ParamHandler handler(THIS_COM"optimization/config/optimization_pitch_setup.yaml");
    std::vector<double> vec_tmp;
    handler.getVector<double>("robot_initial_posture", vec_tmp);
    _ini_body_pos[0] = vec_tmp[0];
    _ini_body_pos[1] = vec_tmp[1];
    _ini_body_pos[2] = vec_tmp[2];

    _ini_body_ori = vec_tmp[3];

    handler.getVector<double>("robot_final_posture", vec_tmp);
    _fin_body_pos[0] = vec_tmp[0];
    _fin_body_pos[1] = vec_tmp[1];
    _fin_body_pos[2] = vec_tmp[2];

    _fin_body_ori = vec_tmp[3];


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

    handler.getValue<double>("landing_loc_region", _landing_loc_region);
    handler.getValue<double>("landing_height_var", _landing_loc_height_var);
}
