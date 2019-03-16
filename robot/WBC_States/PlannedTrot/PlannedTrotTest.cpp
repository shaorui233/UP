#include "PlannedTrotTest.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_States/PlannedTrot/CtrlSet/FullContactCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoLegSwingCtrl.hpp>

#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>

using namespace trot;

template <typename T>
PlannedTrotTest<T>::PlannedTrotTest(FloatingBaseModel<T>* robot, const RobotType & type):
    Test<T>(robot, type)
{
    TEST::_phase = PlannedTrotPhase::lift_up;
    TEST::_state_list.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new FullContactCtrl<T>(this, robot);

    frhl_swing_start_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, 1);
    frhl_swing_ctrl_ = new TwoLegSwingCtrl<T>(this, robot, linkID::FR, linkID::HL);
    frhl_swing_end_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, -1);

    flhr_swing_start_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, 1);
    flhr_swing_ctrl_ = new TwoLegSwingCtrl<T>(this, robot, linkID::FL, linkID::HR);
    flhr_swing_end_trans_ctrl_ = 
        new TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, -1);

    TEST::_state_list.push_back(body_up_ctrl_);

    TEST::_state_list.push_back(body_ctrl_);

    TEST::_state_list.push_back(frhl_swing_start_trans_ctrl_);
    TEST::_state_list.push_back(frhl_swing_ctrl_);
    TEST::_state_list.push_back(frhl_swing_end_trans_ctrl_);

    TEST::_state_list.push_back(body_ctrl_);

    TEST::_state_list.push_back(flhr_swing_start_trans_ctrl_);
    TEST::_state_list.push_back(flhr_swing_ctrl_);
    TEST::_state_list.push_back(flhr_swing_end_trans_ctrl_);

    _SettingParameter();

    _sp = StateProvider<T>::getStateProvider();
    printf("[Trot Test] Constructed\n");
}

template <typename T>
PlannedTrotTest<T>::~PlannedTrotTest(){
    for(size_t i(0); i<Test<T>::_state_list.size(); ++i){
        delete Test<T>::_state_list[i];
    }
}

template <typename T>
void PlannedTrotTest<T>::_TestInitialization(){
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
int PlannedTrotTest<T>::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    //printf("next phase: %i\n", next_phase);

    if(next_phase == PlannedTrotPhase::flhr_swing_start_trans){
    //if(next_phase == PlannedTrotPhase::flhr_swing){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        _sp->_local_frame_global_pos = landing_loc_ave;
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if(next_phase == PlannedTrotPhase::frhl_swing_start_trans){
    //if(next_phase == PlannedTrotPhase::frhl_swing){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::HR];

        _sp->_contact_pt[0] = linkID::FL;
        _sp->_contact_pt[1] = linkID::HR;
        _sp->_num_contact = 2;

        _sp->_local_frame_global_pos = landing_loc_ave;
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if (next_phase == PlannedTrotPhase::NUM_TROT_PHASE) {
        return PlannedTrotPhase::full_contact_1;
    }
    else return next_phase;
}

template <typename T>
void PlannedTrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    while(iter < Test<T>::_state_list.end()){
        (*iter)->SetTestParameter(
                CheetahConfigPath"TEST_trot.yaml");
        ++iter;
    }
}
template <typename T>
void PlannedTrotTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    ext_data->num_step = 5;
}
template class PlannedTrotTest<double>;
template class PlannedTrotTest<float>;
