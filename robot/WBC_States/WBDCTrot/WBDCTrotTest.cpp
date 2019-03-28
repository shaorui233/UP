#include "WBDCTrotTest.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>

#include <WBC_States/WBDCTrot/CtrlSet/WBDCVM_FullContactCtrl.hpp>
#include <WBC_States/WBDCTrot/CtrlSet/WBDCVM_TwoContactTransCtrl.hpp>
#include <WBC_States/WBDCTrot/CtrlSet/WBDCVM_TwoLegSwingCtrl.hpp>

#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/Utilities_print.h>
#include <Utilities/save_file.h>


template <typename T>
WBDCTrotTest<T>::WBDCTrotTest(FloatingBaseModel<T>* robot, const RobotType & type):
    Test<T>(robot, type)
{
    _body_pos.setZero();
    _body_vel.setZero();
    _body_acc.setZero();

    _body_ori_rpy.setZero();
    _body_ang_vel.setZero();

    _front_foot_loc.setZero();
    _hind_foot_loc.setZero();

    Test<T>::_phase = WBDCTrotPhase::lift_up;
    Test<T>::_state_list.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new WBDCVM_FullContactCtrl<T>(this, robot);

    frhl_swing_start_trans_ctrl_ = 
        new WBDCVM_TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, 1);
    frhl_swing_ctrl_ = new WBDCVM_TwoLegSwingCtrl<T>(this, robot, linkID::FR, linkID::HL);
    frhl_swing_end_trans_ctrl_ = 
        new WBDCVM_TwoContactTransCtrl<T>(this, robot, linkID::FR, linkID::HL, -1);

    flhr_swing_start_trans_ctrl_ = 
        new WBDCVM_TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, 1);
    flhr_swing_ctrl_ = new WBDCVM_TwoLegSwingCtrl<T>(this, robot, linkID::FL, linkID::HR);
    flhr_swing_end_trans_ctrl_ = 
        new WBDCVM_TwoContactTransCtrl<T>(this, robot, linkID::FL, linkID::HR, -1);

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

    _ang_vel_filter = new digital_lp_filter<T>(2.*M_PI*20, Test<T>::dt);
    _x_vel_filter = new digital_lp_filter<T>(2.*M_PI*100, Test<T>::dt);
    _y_vel_filter = new digital_lp_filter<T>(2.*M_PI*100, Test<T>::dt);

    _folder_name = "/robot/WBC_States/sim_data/";
    create_folder(_folder_name);
    printf("[WBDC Trot Test] Constructed\n");
}

template <typename T>
WBDCTrotTest<T>::~WBDCTrotTest(){
    for(size_t i(0); i<Test<T>::_state_list.size(); ++i){
        delete Test<T>::_state_list[i];
    }
}

template <typename T>
void WBDCTrotTest<T>::_TestInitialization(){
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

    _vm_q = _sp->_Q;
    _vm_qdot = _sp->_Qdot;
}

template <typename T>
int WBDCTrotTest<T>::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    if(phase == WBDCTrotPhase::lift_up){ // First loop
        _vm_q = _sp->_Q;
        _vm_qdot = _sp->_Qdot;
    }
    if (next_phase == WBDCTrotPhase::NUM_TROT_PHASE) {
        next_phase = WBDCTrotPhase::full_contact_1;
    }
    //printf("next phase: %i\n", next_phase);

   if(next_phase == WBDCTrotPhase::flhr_swing_start_trans){
       Vec3<T> landing_loc_ave = Vec3<T>::Zero();
       landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FR];
       landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HL];
       
       //_body_pos = Test<T>::_state.bodyPosition;
       //_body_pos -= landing_loc_ave;

       _sp->_contact_pt[0] = linkID::FR;
       _sp->_contact_pt[1] = linkID::HL;
       _sp->_num_contact = 2;

       _sp->_local_frame_global_pos.setZero();
   }

    if(next_phase == WBDCTrotPhase::frhl_swing_start_trans){
       Vec3<T> landing_loc_ave = Vec3<T>::Zero();
       landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::FR];
       landing_loc_ave += 0.5 * Test<T>::_robot->_pGC[linkID::HL];
       
       //_body_pos = Test<T>::_state.bodyPosition;
       //_body_pos -= landing_loc_ave;

       _sp->_contact_pt[0] = linkID::FL;
       _sp->_contact_pt[1] = linkID::HR;
       _sp->_num_contact = 2;

       _sp->_local_frame_global_pos.setZero();
    }
    return next_phase;
}

template <typename T>
void WBDCTrotTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter = Test<T>::_state_list.begin();
    ParamHandler* handler = NULL;
    while(iter < Test<T>::_state_list.end()){
        if(Test<T>::_robot_type == RobotType::CHEETAH_3){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_wbdc_trot_cheetah3.yaml");

            handler = new ParamHandler(CheetahConfigPath"TEST_wbdc_trot_cheetah3.yaml");
        }else if (Test<T>::_robot_type == RobotType::MINI_CHEETAH){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_wbdc_trot_mini_cheetah.yaml");
            handler = new ParamHandler(CheetahConfigPath"TEST_wbdc_trot_mini_cheetah.yaml");
        }else{
            printf("[WBDC Trot Test] Invalid robot type\n");
        } 
        ++iter;
    }
    delete handler;
}

template <typename T>
void WBDCTrotTest<T>::_UpdateTestOneStep(){
    // Update Desired Position & Velocity & Acceleration

    if(Test<T>::_phase < 0){ // disabled
        printf("robot is updated with virtual config\n");
        for(size_t i(0); i < cheetah::num_act_joint; ++i){
            Test<T>::_state.q[i] = _vm_q[i + 6];
            Test<T>::_state.qd[i] = _vm_qdot[i + 6];
        }
        Test<T>::_robot->setState(Test<T>::_state);
        Test<T>::_robot->forwardKinematics();

        Vec3<T> ave_foot;
        Vec3<T> ave_foot_vel;
        ave_foot.setZero();
        ave_foot_vel.setZero();

        // Simulation) Update global location
        for(size_t i(0); i<_sp->_num_contact; ++i){
            ave_foot += (1./_sp->_num_contact) * Test<T>::_robot->_pGC[_sp->_contact_pt[i] ];
            ave_foot_vel += (1./_sp->_num_contact) * Test<T>::_robot->_vGC[_sp->_contact_pt[i]];
        }
        Test<T>::_state.bodyPosition -= ave_foot;
        Test<T>::_state.bodyVelocity.tail(3) -= ave_foot_vel;

        // Update with new body position
        Test<T>::_robot->setState(Test<T>::_state);
        Test<T>::_robot->forwardKinematics();

        // Update Mass, Gravity, Coriolis
        Test<T>::_robot->contactJacobians();
        Test<T>::_robot->massMatrix();
        Test<T>::_robot->gravityForce();
        Test<T>::_robot->coriolisForce();
    }

    //T scale(1.0);
    if(Test<T>::_phase == WBDCTrotPhase::frhl_swing || 
            Test<T>::_phase == WBDCTrotPhase::flhr_swing){
        _body_ang_vel.setZero();
    }else{
        _body_ang_vel[2] = _sp->_ori_command[2];
        _body_ori_rpy[2] += _body_ang_vel[2]*Test<T>::dt;

        Mat3<T> Rot = rpyToRotMat(_body_ori_rpy);
        Vec3<T> input_vel; input_vel.setZero();
        input_vel[0] = _sp->_dir_command[0];
        //_body_vel[1] = -scale*_sp->_dir_command[1];
        _body_vel = Rot.transpose() * input_vel;
        //_body_vel = Rot * _body_vel;
    }
    //_body_pos += _body_vel*Test<T>::dt;

    static int count(0);
    if(count % 10 == 0){
        saveValue(_sp->_curr_time, _folder_name, "time");
        saveVector(_body_pos, _folder_name, "body_pos");
        saveVector(_body_vel, _folder_name, "body_vel");
        saveVector(_body_acc, _folder_name, "body_acc");
        saveVector(_body_ori_rpy, _folder_name, "body_ori_rpy");
        saveVector(_body_ang_vel, _folder_name, "body_ang_vel");

        saveVector(_sp->_Q, _folder_name, "config");
        saveVector(_sp->_Qdot, _folder_name, "qdot");

        saveVector(_vm_q, _folder_name, "vm_q");
        saveVector(_vm_qdot, _folder_name, "vm_qdot");


        saveValue(Test<T>::_phase, _folder_name, "phase");
    }
    ++count;
}

template <typename T>
void WBDCTrotTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    (void)ext_data;
}

template class WBDCTrotTest<double>;
template class WBDCTrotTest<float>;
