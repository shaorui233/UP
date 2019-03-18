#include "PlannedTrotTest.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_States/PlannedTrot/CtrlSet/FullContactCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoLegSwingCtrl.hpp>

#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>
#include <WBC_States/PlannedTrot/Planner/Planner.hpp>

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
    
    _sp = StateProvider<T>::getStateProvider();
    _planner = new Planner<T>();
    _SettingParameter();

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

    if(_planner->stop()){
        if(phase == PlannedTrotPhase::full_contact_1)
            next_phase = PlannedTrotPhase::full_contact_1;
        else if(phase == PlannedTrotPhase::full_contact_2)
            next_phase = PlannedTrotPhase::full_contact_2;
        else
            printf("Robot is moving... cannot stop\n");
     }
    if(next_phase == PlannedTrotPhase::flhr_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        //_sp->_local_frame_global_pos = landing_loc_ave;
        _sp->_local_frame_global_pos.setZero();
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if(next_phase == PlannedTrotPhase::frhl_swing_start_trans){
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * TEST::_robot->_pGC[linkID::HR];

        _sp->_contact_pt[0] = linkID::FL;
        _sp->_contact_pt[1] = linkID::HR;
        _sp->_num_contact = 2;

        //_sp->_local_frame_global_pos = landing_loc_ave;
        _sp->_local_frame_global_pos.setZero();
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if (next_phase == PlannedTrotPhase::NUM_TROT_PHASE) {
        return PlannedTrotPhase::full_contact_1;
    }
    else { return next_phase; }
}

template <typename T>
void PlannedTrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    ParamHandler* handler = NULL;
    while(iter < Test<T>::_state_list.end()){
        if(TEST::_robot_type == RobotType::CHEETAH_3){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");

            handler = new ParamHandler(CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");
            _planner->SetParameter(CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");
        }else if (TEST::_robot_type == RobotType::MINI_CHEETAH){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
            handler = new ParamHandler(CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
            _planner->SetParameter(CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
        }else{
            printf("[Planned Trot Test] Invalid robot type\n");
        } 
        ++iter;
    }
    // Timing Parameter
    _step_time = 0.;
    T t;
    handler->getValue<T>("swing_time", t);
    _step_time += t;
    handler->getValue<T>("transition_time", t);
    _step_time += (2.*t);
    handler->getValue<T>("stance_time", t);
    _step_time += t;
    _tot_time = _step_time * (double)(_planner->nMiddle+1);

    delete handler;
}
template <typename T>
void PlannedTrotTest<T>::_UpdateTestOneStep(){
    // Update User Input
    _planner->UpdateUserInput(_sp->_dir_command, _sp->_ori_command);
    // Update Desired Position & Velocity & Acceleration
    // Update Desired Orientation 

}
template <typename T>
void PlannedTrotTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    ext_data->num_step = 0;

    size_t resolution(50);
    ext_data->num_path_pt = resolution + 1;

    T dt = _tot_time/((double)resolution);
    T pos[3];
    pos[0] =0.;
    pos[1] =0.;
    pos[2] =0.;
    for(size_t i(0); i<resolution + 1; ++i){
        _planner->_pos_spline.getCurvePoint(dt * i, pos);
        ext_data->path_x[i] = pos[0];
        ext_data->path_y[i] = pos[1];
        ext_data->path_z[i] = pos[2];
    }

    // 
    ext_data->num_middle_pt = _planner->nMiddle + 2;
    T ori[3];
    ori[0] =0.;
    ori[1] =0.;
    ori[2] =0.;
    for(size_t i(0); i<_planner->nMiddle + 2; ++i){
        _planner->_pos_spline.getCurvePoint(_step_time* i, pos);
        _planner->_ori_spline.getCurvePoint(_step_time* i, ori);

        ext_data->mid_ori_roll[i] = ori[0];
        ext_data->mid_ori_pitch[i] = ori[1];
        ext_data->mid_ori_yaw[i] = ori[2];

        ext_data->mid_x[i] = pos[0];
        ext_data->mid_y[i] = pos[1];
        ext_data->mid_z[i] = pos[2];
    }
}
template class PlannedTrotTest<double>;
template class PlannedTrotTest<float>;
