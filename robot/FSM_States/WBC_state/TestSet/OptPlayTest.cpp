#include "OptPlayTest.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/CtrlSet/BodyCtrl.hpp>

#include <WBC_state/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_state/CtrlSet/TwoLegSwingCtrl.hpp>

#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Dynamics/Quadruped.h>

#include <Utilities/Utilities_print.h>
#include <WBC_state/OptInterpreter.hpp>

template <typename T>
OptPlayTest<T>::OptPlayTest(const FloatingBaseModel<T>* robot):Test<T>(),
    _robot_sys(robot)
{
    Test<T>::phase_ = OptPlayPhase::stance_wait;
    Test<T>::state_list_.clear();

    body_ctrl_stay_ = new BodyCtrl<T>(robot);
    body_ctrl_ = new BodyCtrl<T>(robot);

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

    Test<T>::state_list_.push_back(body_ctrl_stay_);

    Test<T>::state_list_.push_back(body_ctrl_);

    Test<T>::state_list_.push_back(frhl_swing_start_trans_ctrl_);
    Test<T>::state_list_.push_back(frhl_swing_ctrl_);
    Test<T>::state_list_.push_back(frhl_swing_end_trans_ctrl_);

    Test<T>::state_list_.push_back(body_ctrl_);

    Test<T>::state_list_.push_back(flhr_swing_start_trans_ctrl_);
    Test<T>::state_list_.push_back(flhr_swing_ctrl_);
    Test<T>::state_list_.push_back(flhr_swing_end_trans_ctrl_);

    _SettingParameter();

    _sp = Cheetah_StateProvider<T>::getStateProvider();
    printf("[OptPlay Test] Constructed\n");
}

template <typename T>
OptPlayTest<T>::~OptPlayTest(){
    for(size_t i(0); i<Test<T>::state_list_.size(); ++i){
        delete Test<T>::state_list_[i];
    }
}

template <typename T>
void OptPlayTest<T>::TestInitialization(){
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
        _sp->curr_time_ = 0.; // Reset current time
        _sp->_opt_play = true;
    }

    if(next_phase == OptPlayPhase::flhr_swing_start_trans){
    //if(next_phase == OptPlayPhase::flhr_swing){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        _sp->_local_frame_global_pos = landing_loc_ave;
        pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
        ++_sp->_num_step;
    printf("num_step: %d\n", _sp->_num_step);
    }

    if(next_phase == OptPlayPhase::frhl_swing_start_trans){
    //if(next_phase == OptPlayPhase::frhl_swing){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::HR];

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
        //_sp->_opt_play = false;
        _sp->_opt_play = true;
    }
    
    //printf("next phase: %i\n", next_phase);

    if (next_phase == OptPlayPhase::NUM_OPT_PHASE) {
        return OptPlayPhase::full_contact_1;
    }
    else return next_phase;
}

template <typename T>
void OptPlayTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::state_list_.begin();
    while(iter < Test<T>::state_list_.end()){
        (*iter)->SetTestParameter(
                CheetahConfigPath"TEST_opt_play.yaml");
        ++iter;
    }
    ParamHandler handler(CheetahConfigPath"TEST_opt_play.yaml");
    
    T tmp(0.);
    handler.getValue<T>("initial_stance_time", tmp);
    ((BodyCtrl<T>*)body_ctrl_stay_)->SetStanceTime(tmp);

    handler.getValue<int>("max_num_step", _max_num_step);
    
    OptInterpreter<T>::getOptInterpreter()->SetParameter(CheetahConfigPath"TEST_opt_play.yaml");

}

template class OptPlayTest<double>;
template class OptPlayTest<float>;
