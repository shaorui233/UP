#include "PlannedTrotTest.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_States/PlannedTrot/CtrlSet/FullContactCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoContactTransCtrl.hpp>
#include <WBC_States/PlannedTrot/CtrlSet/TwoLegSwingCtrl.hpp>

#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>
#include <Utilities/save_file.h>
#include <WBC_States/PlannedTrot/Planner/Planner.hpp>
#include <WBC_States/PlannedTrot/Planner/Path.hpp>

using namespace trot;

template <typename T>
PlannedTrotTest<T>::PlannedTrotTest(FloatingBaseModel<T>* robot, const RobotType & type):
    Test<T>(robot, type)
{
    Test<T>::_phase = PlannedTrotPhase::lift_up;
    Test<T>::_state_list.clear();

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

    Test<T>::_state_list.push_back(body_up_ctrl_);

    Test<T>::_state_list.push_back(body_ctrl_);

    Test<T>::_state_list.push_back(frhl_swing_start_trans_ctrl_);
    Test<T>::_state_list.push_back(frhl_swing_ctrl_);
    Test<T>::_state_list.push_back(frhl_swing_end_trans_ctrl_);

    Test<T>::_state_list.push_back(body_ctrl_);

    Test<T>::_state_list.push_back(flhr_swing_start_trans_ctrl_);
    Test<T>::_state_list.push_back(flhr_swing_ctrl_);
    Test<T>::_state_list.push_back(flhr_swing_end_trans_ctrl_);
    
    _sp = StateProvider<T>::getStateProvider();
    _planner = new Planner<T>(stance_foot::FLHR);
    _SettingParameter();


    _folder_name = "/robot/WBC_States/sim_data/";
    create_folder(_folder_name);
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
    if (next_phase == PlannedTrotPhase::NUM_TROT_PHASE) {
        next_phase = PlannedTrotPhase::full_contact_1;
    }
    //printf("next phase: %i\n", next_phase);

    // Which path segment we are going to use in this swing
    if(next_phase == PlannedTrotPhase::flhr_swing_start_trans|| 
            next_phase == PlannedTrotPhase::frhl_swing_start_trans){
        _planner->updateStepIdx(_sp->_curr_time);
    }
    // Check whether Cheetah must stop or step
    if(_planner->stop(_sp->_curr_time)){
        if(phase == PlannedTrotPhase::full_contact_1)
            next_phase = PlannedTrotPhase::full_contact_1;
      
        else if(phase == PlannedTrotPhase::full_contact_2)
            next_phase = PlannedTrotPhase::full_contact_2;
        else
            printf("Robot was in phase %d, stop in the following phase\n", Test<T>::_phase);
     }
    if(next_phase == PlannedTrotPhase::flhr_swing_start_trans){
        _planner->getNextFootLocation(stance_foot::FRHL,
                _front_foot_loc, _hind_foot_loc);
       
        //pretty_print(_front_foot_loc, std::cout, "front foot");
        //pretty_print(_hind_foot_loc, std::cout, "hind foot");
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HL];

        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;
        //_sp->_local_frame_global_pos = landing_loc_ave;
        _sp->_local_frame_global_pos.setZero();
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if(next_phase == PlannedTrotPhase::frhl_swing_start_trans){
        _planner->getNextFootLocation(stance_foot::FLHR,
                _front_foot_loc, _hind_foot_loc);

        //pretty_print(_front_foot_loc, std::cout, "front foot");
        //pretty_print(_hind_foot_loc, std::cout, "hind foot");
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HR];

        _sp->_contact_pt[0] = linkID::FL;
        _sp->_contact_pt[1] = linkID::HR;
        _sp->_num_contact = 2;

        //_sp->_local_frame_global_pos = landing_loc_ave;
        _sp->_local_frame_global_pos.setZero();
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }
    return next_phase;
}

template <typename T>
void PlannedTrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    ParamHandler* handler = NULL;
    while(iter < Test<T>::_state_list.end()){
        if(Test<T>::_robot_type == RobotType::CHEETAH_3){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");

            handler = new ParamHandler(CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");
            _planner->SetParameter(CheetahConfigPath"TEST_planned_trot_cheetah3.yaml");
        }else if (Test<T>::_robot_type == RobotType::MINI_CHEETAH){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
            handler = new ParamHandler(CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
            _planner->SetParameter(CheetahConfigPath"TEST_planned_trot_mini_cheetah.yaml");
        }else{
            printf("[Planned Trot Test] Invalid robot type\n");
        } 
        ++iter;
    }
    delete handler;
}

template <typename T>
void PlannedTrotTest<T>::_UpdateTestOneStep(){
    // Update User Input
    _planner->UpdateUserInput(_sp->_dir_command, _sp->_ori_command);
    // Update Desired Position & Velocity & Acceleration
    _planner->getBodyConfig(_sp->_curr_time, 
            _body_pos, _body_vel, _body_acc, 
            _body_ori_rpy, _body_ang_vel);

    static int count(0);
    if(count % 10 == 0){
        saveValue(_sp->_curr_time, _folder_name, "time");
        saveVector(_body_pos, _folder_name, "body_pos");
        saveVector(_body_vel, _folder_name, "body_vel");
        saveVector(_body_acc, _folder_name, "body_acc");
        saveVector(_body_ori_rpy, _folder_name, "cmd_body_ori_rpy");
        // Body ori
        Vec3<T> body_ori = ori::quatToRPY(Test<T>::_robot->_state.bodyOrientation);
        saveVector(body_ori, _folder_name, "body_ori_rpy");
        saveVector(_body_ang_vel, _folder_name, "body_ang_vel");

        saveVector(_sp->_Q, _folder_name, "config");
        saveVector(_sp->_Qdot, _folder_name, "qdot");

        saveValue(Test<T>::_phase, _folder_name, "phase");
    }
    ++count;
}

template <typename T>
void PlannedTrotTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    _planner->updateExtraData(ext_data);
}

template class PlannedTrotTest<double>;
template class PlannedTrotTest<float>;
