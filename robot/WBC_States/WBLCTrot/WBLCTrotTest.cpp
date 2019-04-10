#include "WBLCTrotTest.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_States/WBLCTrot/CtrlSet/WBLC_FullContactCtrl.hpp>
#include <WBC_States/WBLCTrot/CtrlSet/WBLC_TwoContactTransCtrl.hpp>
#include <WBC_States/WBLCTrot/CtrlSet/WBLC_TwoLegSwingCtrl.hpp>

#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>
#include <Utilities/save_file.h>

template <typename T>
WBLCTrotTest<T>::WBLCTrotTest(FloatingBaseModel<T>* robot, const RobotType & type):
    Test<T>(robot, type)
{
    Test<T>::_phase = WBLCTrotPhase::lift_up;
    Test<T>::_state_list.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new WBLC_FullContactCtrl<T>(this, robot);

    frhl_swing_start_trans_ctrl_ = 
        new WBLC_TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, 1);
    frhl_swing_ctrl_ = new WBLC_TwoLegSwingCtrl<T>(this, robot, linkID::FR, linkID::HL);
    frhl_swing_end_trans_ctrl_ = 
        new WBLC_TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, -1);

    flhr_swing_start_trans_ctrl_ = 
        new WBLC_TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, 1);
    flhr_swing_ctrl_ = new WBLC_TwoLegSwingCtrl<T>(this, robot, linkID::FL, linkID::HR);
    flhr_swing_end_trans_ctrl_ = 
        new WBLC_TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, -1);

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
    _SettingParameter();

    // (x, y)
    _filtered_input_vel.push_back(new digital_lp_filter<T>(2.*M_PI*15. , Test<T>::dt));
    _filtered_input_vel.push_back(new digital_lp_filter<T>(2.*M_PI*15. , Test<T>::dt));

    _input_vel.setZero();
        
    _folder_name = "/robot/WBC_States/sim_data/";
    create_folder(_folder_name);
    printf("[Trot Test] Constructed\n");
}

template <typename T>
WBLCTrotTest<T>::~WBLCTrotTest(){
    for(size_t i(0); i<Test<T>::_state_list.size(); ++i){
        delete Test<T>::_state_list[i];
    }
}

template <typename T>
void WBLCTrotTest<T>::_TestInitialization(){
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
int WBLCTrotTest<T>::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    if (next_phase == WBLCTrotPhase::NUM_TROT_PHASE) {
        next_phase = WBLCTrotPhase::full_contact_1;
    }
    //printf("next phase: %i\n", next_phase);

    // Which path segment we are going to use in this swing
    if(next_phase == WBLCTrotPhase::flhr_swing_start_trans|| 
            next_phase == WBLCTrotPhase::frhl_swing_start_trans){
    }
    // Check whether Cheetah must stop or step
   if(next_phase == WBLCTrotPhase::flhr_swing_start_trans){
       
        //pretty_print(_front_foot_loc, std::cout, "front foot");
        //pretty_print(_hind_foot_loc, std::cout, "hind foot");
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FR];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HL];

        landing_loc_ave[2] = 0.; // Flat terrain
        _body_pos -= landing_loc_ave;
        _sp->_contact_pt[0] = linkID::FR;
        _sp->_contact_pt[1] = linkID::HL;
        _sp->_num_contact = 2;

        //_sp->_local_frame_global_pos = landing_loc_ave;
        _sp->_local_frame_global_pos.setZero();
        //pretty_print(_sp->_local_frame_global_pos, std::cout, "local frame");
    }

    if(next_phase == WBLCTrotPhase::frhl_swing_start_trans){

        //pretty_print(_front_foot_loc, std::cout, "front foot");
        //pretty_print(_hind_foot_loc, std::cout, "hind foot");
        Vec3<T> landing_loc_ave = Vec3<T>::Zero();
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FL];
        landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HR];

        landing_loc_ave[2] = 0.; // Flat terrain
        _body_pos -= landing_loc_ave;
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
void WBLCTrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    ParamHandler* handler = NULL;
    while(iter < Test<T>::_state_list.end()){
        if(Test<T>::_robot_type == RobotType::CHEETAH_3){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_wblc_trot_cheetah3.yaml");

            handler = new ParamHandler(CheetahConfigPath"TEST_wblc_trot_cheetah3.yaml");
        }else if (Test<T>::_robot_type == RobotType::MINI_CHEETAH){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_wblc_trot_mini_cheetah.yaml");
            handler = new ParamHandler(CheetahConfigPath"TEST_wblc_trot_mini_cheetah.yaml");
        }else{
            printf("[WBLC Trot Test] Invalid robot type\n");
        } 
        ++iter;
    }
    handler->getValue<T>("body_height", _target_body_height);
    _body_pos[2] = _target_body_height;
    handler->getBoolean("save_file", Test<T>::_b_save_file);
    delete handler;
}

template <typename T>
void WBLCTrotTest<T>::_UpdateTestOneStep(){
    T scale(4.0);
    if(Test<T>::_phase == WBLCTrotPhase::frhl_swing || 
            Test<T>::_phase == WBLCTrotPhase::flhr_swing){
        _body_ang_vel.setZero();
        _filtered_input_vel[0]->input(_input_vel[0]);
        _filtered_input_vel[1]->input(_input_vel[1]);
     }else{
        _body_ang_vel[2] = _sp->_ori_command[2];
        _body_ori_rpy[2] += _body_ang_vel[2]*Test<T>::dt;

        
        _filtered_input_vel[0]->input(scale*_sp->_dir_command[0]);
        _filtered_input_vel[1]->input(-0.5*scale*_sp->_dir_command[1]);
        _input_vel[0] = _filtered_input_vel[0]->output();
        _input_vel[1] = _filtered_input_vel[1]->output();
        
        Mat3<T> Rot = rpyToRotMat(_body_ori_rpy);
        _body_vel = Rot.transpose() * _input_vel;
    }
    _body_pos += _body_vel*Test<T>::dt;
    //_body_pos = Test<T>::_robot->_state.bodyPosition;
    
}

template <typename T>
void WBLCTrotTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    (void)ext_data;

    if(Test<T>::_b_save_file){
        static int count(0);
        if(count % 10 == 0){
            saveValue(_sp->_curr_time, _folder_name, "time");
            saveVector(_body_pos, _folder_name, "body_pos");
            saveVector(_body_vel, _folder_name, "body_vel");
            saveVector(_body_acc, _folder_name, "body_acc");

            Vec3<T> body_ori_rpy = ori::quatToRPY(Test<T>::_robot->_state.bodyOrientation);
            saveVector(body_ori_rpy, _folder_name, "body_ori_rpy");
            saveVector(_body_ang_vel, _folder_name, "body_ang_vel");
            saveVector(_body_ori_rpy, _folder_name, "cmd_body_ori_rpy");

            saveVector(_sp->_Q, _folder_name, "config");
            saveVector(_sp->_Qdot, _folder_name, "qdot");
            saveVector((Test<T>::_copy_cmd)[0].tauFeedForward, _folder_name, "fr_tau");
            saveVector((Test<T>::_copy_cmd)[1].tauFeedForward, _folder_name, "fl_tau");
            saveVector((Test<T>::_copy_cmd)[2].tauFeedForward, _folder_name, "hr_tau");
            saveVector((Test<T>::_copy_cmd)[3].tauFeedForward, _folder_name, "hl_tau");

            saveValue(Test<T>::_phase, _folder_name, "phase");
        }
        ++count;
    }
}

template class WBLCTrotTest<double>;
template class WBLCTrotTest<float>;
