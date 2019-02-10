#include "Cheetah_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>

#include "Cheetah_StateProvider.hpp"
#include <ParamHandler.hpp>

// Body Ctrl Test
#include <WBC_state/TestSet/BodyCtrlTest.hpp>
#include <WBC_state/TestSet/JPosCtrlTest.hpp>
#include <WBC_state/TestSet/TrotTest.hpp>
#include <Utilities/Utilities_print.h>

template <typename T>
Cheetah_interface<T>::Cheetah_interface(FloatingBaseModel<T> * robot):
    _robot(robot),
    count_(0),
    waiting_count_(100),
    running_time_(0.)
{
    _sp = Cheetah_StateProvider<T>::getStateProvider();
    _ParameterSetting();
    _state.q = DVec<T>::Zero(cheetah::num_act_joint);
    _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
    printf("[Cheetah_interface] Contruct\n");
}

template <typename T>
Cheetah_interface<T>::~Cheetah_interface(){
    delete _test;
}

template <typename T>
void Cheetah_interface<T>::GetCommand(const Cheetah_Data<T>* data, 
        LegControllerCommand<T> * command){
    for(size_t i(0); i < cheetah::num_act_joint; ++i){
        _state.q[i] = data->jpos[i];
        _state.qd[i] = data->jvel[i];
        
        _sp->Q_[i+6] = data->jpos[i];
        _sp->Qdot_[i+6] = data->jvel[i];
    }
    for(size_t i(0); i<4; ++i){
        _state.bodyOrientation[i] = data->body_ori[i];
    }

    _state.bodyPosition.setZero();
    _state.bodyVelocity.setZero();
    _state.bodyVelocity[0] = data->ang_vel[0];
    _state.bodyVelocity[1] = data->ang_vel[1];
    _state.bodyVelocity[2] = data->ang_vel[2];

    _robot->setState(_state);
    _robot->forwardKinematics();

    // TEST
    Vec3<T> ave_foot;
    ave_foot.setZero();

    for(size_t i(0); i<_sp->_num_contact; ++i){
        ave_foot += (1./_sp->_num_contact) * _robot->_pGC[_sp->_contact_pt[i]];
    }
    //ave_foot += 0.25 *_robot->_pGC[linkID::FR];
    //ave_foot += 0.25 *_robot->_pGC[linkID::FL];
    //ave_foot += 0.25 *_robot->_pGC[linkID::HL];
    //ave_foot += 0.25 *_robot->_pGC[linkID::HR];

    
    //for(size_t i(0); i<_robot->_pGC.size(); ++i){
    //pretty_print(_robot->_pGC[i], std::cout, "contact position ");
    //}
    //pretty_print(_robot->_pGC[linkID::FR], std::cout, "FR");
    //pretty_print(_robot->_pGC[linkID::FL], std::cout, "FL");
    //pretty_print(_robot->_pGC[linkID::HR], std::cout, "HR");
    //pretty_print(_robot->_pGC[linkID::HL], std::cout, "HL");

    //pretty_print(_state.bodyOrientation, std::cout, "body ori");
    //pretty_print(data->body_ori, "data body ori", 4);

    //printf("joystick command: %f, %f \n", data->dir_command[0], data->dir_command[1]);
    _sp->_dir_command[0] = data->dir_command[0];
    _sp->_dir_command[1] = data->dir_command[1];
    _state.bodyPosition = -ave_foot;// + _sp->_local_frame_global_pos;
    
    // Update with new body position
    _robot->setState(_state);
    _robot->forwardKinematics();

    //pretty_print(_state.bodyPosition, std::cout, "body position");

    // Update Mass, Gravity, Coriolis
    _robot->contactJacobians();
    _robot->massMatrix();
    _robot->gravityForce();
    _robot->coriolisForce();

    if(!_Initialization(data, command)){
        _test->getCommand(command);
    }
    
   running_time_ = (T)(count_) * cheetah::servo_rate;
    ++count_;
    // When there is sensed time
    _sp->curr_time_ = running_time_;
}

template <typename T>
bool Cheetah_interface<T>::_Initialization(
        const Cheetah_Data<T>* data, LegControllerCommand<T>* command){

    static bool test_initialized(false);
    if(!test_initialized) {
        _test->TestInitialization();
        test_initialized = true;
        printf("[Cheetah Interface] Test initialization is done\n");
    }
    if(count_ < waiting_count_){
        for(size_t leg(0); leg<cheetah::num_leg; ++leg){
            for(size_t jidx(0); jidx<cheetah::num_leg_joint; ++jidx){
                command[leg].tauFeedForward[jidx] = 0.;
                command[leg].qDes[jidx] = data->jpos[3*leg + jidx];
                command[leg].qdDes[jidx] = 0.;
            }
        }
        return true;
    }
    return false;
}

template <typename T>
void Cheetah_interface<T>::_ParameterSetting(){
    ParamHandler handler(CheetahConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Walking Test ***********************************
    if(tmp_string == "trot_test"){
        _test = new TrotTest<T>(_robot);
    // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        _test = new BodyCtrlTest<T>(_robot);    
    // JPos Ctrl Test ************************************
    }else if(tmp_string == "jpos_ctrl_test"){
        _test = new JPosCtrlTest<T>(_robot);    
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    printf("[Cheetah_interface] Parameter setup is done\n");
}

template class Cheetah_interface<double>;
template class Cheetah_interface<float>;


