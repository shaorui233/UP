#include "OptPlayTest.hpp"

#include <WBC_States/OptPlay/CtrlSet/FullContactCtrl.hpp>
#include <WBC_States/OptPlay/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_States/OptPlay/CtrlSet/TwoLegSwingCtrl.hpp>
#include <WBC_States/OptPlay/OptInterpreter.hpp>

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>

template <typename T>
OptPlayTest<T>::OptPlayTest(FloatingBaseModel<T>* robot, const RobotType & robot_type):
    Test<T>(robot, robot_type){

        Test<T>::_phase = OptPlayPhase::stance_wait;
        Test<T>::_state_list.clear();

        body_ctrl_stay_ = new FullContactCtrl<T>(robot);
        body_ctrl_ = new FullContactCtrl<T>(robot);

        frhl_swing_start_trans_ctrl_ = 
            new TwoContactTransCtrl<T>(robot, linkID::FR, linkID::HL, 1);
        frhl_swing_ctrl_ = new TwoLegSwingCtrl<T>(robot, linkID::FR, linkID::HL);
        frhl_swing_end_trans_ctrl_ = 
            new TwoContactTransCtrl<T>(robot, linkID::FR, linkID::HL, -1);

        flhr_swing_start_trans_ctrl_ = 
            new TwoContactTransCtrl<T>(robot, linkID::FL, linkID::HR, 1);
        flhr_swing_ctrl_ = new TwoLegSwingCtrl<T>(robot, linkID::FL, linkID::HR);
        flhr_swing_end_trans_ctrl_ = 
            new TwoContactTransCtrl<T>(robot, linkID::FL, linkID::HR, -1);

        Test<T>::_state_list.push_back(body_ctrl_stay_);

        Test<T>::_state_list.push_back(body_ctrl_);

        Test<T>::_state_list.push_back(frhl_swing_start_trans_ctrl_);
        Test<T>::_state_list.push_back(frhl_swing_ctrl_);
        Test<T>::_state_list.push_back(frhl_swing_end_trans_ctrl_);

        Test<T>::_state_list.push_back(body_ctrl_);

        Test<T>::_state_list.push_back(flhr_swing_start_trans_ctrl_);
        Test<T>::_state_list.push_back(flhr_swing_ctrl_);
        Test<T>::_state_list.push_back(flhr_swing_end_trans_ctrl_);

        _SettingParameter();

        _sp = StateProvider<T>::getStateProvider();
        printf("[OptPlay Test] Constructed\n");
    }

template <typename T>
OptPlayTest<T>::~OptPlayTest(){
    for(size_t i(0); i<Test<T>::_state_list.size(); ++i){
        delete Test<T>::_state_list[i];
    }
}

template <typename T>
void OptPlayTest<T>::_TestInitialization(){
    // Yaml file name
    body_ctrl_stay_->CtrlInitialization("CTRL_fix_stance");
    body_ctrl_->CtrlInitialization("CTRL_fix_stance");

    // Transition
    frhl_swing_start_trans_ctrl_->CtrlInitialization("CTRL_two_leg_trans");
    frhl_swing_end_trans_ctrl_->CtrlInitialization("CTRL_two_leg_trans");
    flhr_swing_start_trans_ctrl_->CtrlInitialization("CTRL_two_leg_trans");
    flhr_swing_end_trans_ctrl_->CtrlInitialization("CTRL_two_leg_trans");

    // Swing
    frhl_swing_ctrl_->CtrlInitialization("CTRL_frhl_swing");
    flhr_swing_ctrl_->CtrlInitialization("CTRL_flhr_swing");
}

template <typename T>
int OptPlayTest<T>::_NextPhase(const int & phase){
    int next_phase = phase + 1;


    if( (next_phase == OptPlayPhase::full_contact_1) && (_sp->_num_step < 0) ){
        _sp->_curr_time = 0.; // Reset current time
    }

    if(next_phase == OptPlayPhase::flhr_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        _sp->_local_frame_global_pos = landing_loc_ave;
        pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
        ++_sp->_num_step;
        printf("num_step: %d\n", _sp->_num_step);
    }

    if(next_phase == OptPlayPhase::frhl_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HR];

        _sp->_contact_pt[0] = linkID::FL;
        _sp->_contact_pt[1] = linkID::HR;
        _sp->_num_contact = 2;

        _sp->_local_frame_global_pos = landing_loc_ave;
        pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
        ++_sp->_num_step;
        printf("num_step: %d\n", _sp->_num_step);
    }

    // Stay when it finishes
    if(_sp->_num_step > _max_num_step){ 
        next_phase = OptPlayPhase::stance_wait; 
    }

    //printf("next phase: %i\n", next_phase);

    if (next_phase == OptPlayPhase::NUM_OPT_PHASE) {
        return OptPlayPhase::full_contact_1;
    }
    else return next_phase;
}

template <typename T>
void OptPlayTest<T>::_SettingParameter(){
    if(Test<T>::_robot_type == RobotType::MINI_CHEETAH){
        printf("No mini cheetah data\n");
        exit(0);
    }
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    while(iter < Test<T>::_state_list.end()){
        (*iter)->SetTestParameter(
                CheetahConfigPath"TEST_opt_play_cheetah3.yaml");
        ++iter;
    }
    ParamHandler handler(CheetahConfigPath"TEST_opt_play_cheetah3.yaml");
    handler.getValue<int>("max_num_step", _max_num_step);
    OptInterpreter<T>::getOptInterpreter()->
        SetParameter(CheetahConfigPath"TEST_opt_play_cheetah3.yaml");
}

template<typename T>
void OptPlayTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    OptInterpreter<T> *  inter = OptInterpreter<T>::getOptInterpreter();
    static size_t num_planned_foot_step = inter->_foot_step_list.size();

    size_t start_idx = 0;
    if(_sp->_num_step > 0 ) start_idx = _sp->_num_step;
    size_t j(0);
    for(size_t i(start_idx); i<num_planned_foot_step; ++i){
        ext_data->loc_x[2*j] = inter->_foot_step_list[i][0]; 
        ext_data->loc_x[2*j + 1] = inter->_foot_step_list[i][3]; 

        ext_data->loc_y[2*j] = inter->_foot_step_list[i][1]; 
        ext_data->loc_y[2*j + 1] = inter->_foot_step_list[i][4]; 

        ext_data->loc_z[2*j] = inter->_foot_step_list[i][2]; 
        ext_data->loc_z[2*j + 1] = inter->_foot_step_list[i][5];

        //pretty_print(inter->_foot_step_list[i], std::cout, "foot step");
        ++j;
    }
    //pretty_print(ext_data->loc_x, "loc x", 2*num_planned_foot_step);
    //printf("\n");
    //pretty_print(ext_data->loc_y, "loc y", 2*num_planned_foot_step);
    //printf("\n");
    //pretty_print(ext_data->loc_z, "loc z", 2*num_planned_foot_step);
    //printf("\n");

    ext_data->num_step = 2*(num_planned_foot_step - _sp->_num_step);
}


template class OptPlayTest<double>;
template class OptPlayTest<float>;
