#include "TrotTest.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/CtrlSet/BodyCtrl.hpp>
#include <WBC_state/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_state/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_state/CtrlSet/TwoLegSwingCtrl.hpp>

#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Dynamics/Quadruped.h>

    template <typename T>
TrotTest<T>::TrotTest(const FloatingBaseModel<T>* robot):Test<T>(),
    _robot_sys(robot)
{
    Test<T>::phase_ = TrotPhase::lift_up;
    Test<T>::state_list_.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new BodyCtrl<T>(robot);

    frhl_swing_start_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(robot, linkID::FR, linkID::HL, -1);
    frhl_swing_ctrl_ = new TwoLegSwingCtrl<T>(robot, linkID::FR, linkID::HL);
    frhl_swing_end_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(robot, linkID::FR, linkID::HL, 1);

    flhr_swing_start_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(robot, linkID::FL, linkID::HR, -1);
    flhr_swing_ctrl_ = new TwoLegSwingCtrl<T>(robot, linkID::FL, linkID::HR);
    flhr_swing_end_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(robot, linkID::FL, linkID::HR, 1);

    Test<T>::state_list_.push_back(body_up_ctrl_);

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
    printf("[Trot Test] Constructed\n");
}

template <typename T>
TrotTest<T>::~TrotTest(){
    for(size_t i(0); i<Test<T>::state_list_.size(); ++i){
        delete Test<T>::state_list_[i];
    }
}

template <typename T>
void TrotTest<T>::TestInitialization(){
    // Yaml file name
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
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
int TrotTest<T>::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    //printf("next phase: %i\n", next_phase);

    if(next_phase == TrotPhase::flhr_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        //_sp->_local_frame_global_pos.head(2) += landing_loc_ave.head(2);
    }

    if(next_phase == TrotPhase::frhl_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * _robot_sys->_pGC[linkID::HR];

        _sp->_contact_pt[0] = linkID::FL;
        _sp->_contact_pt[1] = linkID::HR;
        _sp->_num_contact = 2;
        //_sp->_local_frame_global_pos.head(2) += landing_loc_ave.head(2);
    }



    if (next_phase == TrotPhase::NUM_TROT_PHASE) {
        return TrotPhase::full_contact_1;
    }
    else return next_phase;
}

template <typename T>
void TrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::state_list_.begin();
    while(iter < Test<T>::state_list_.end()){
        (*iter)->SetTestParameter(
                CheetahConfigPath"TEST_trot.yaml");
        ++iter;
    }
}

template class TrotTest<double>;
template class TrotTest<float>;
