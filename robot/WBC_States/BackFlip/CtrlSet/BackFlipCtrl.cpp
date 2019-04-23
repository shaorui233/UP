#include "BackFlipCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/WBLCTrot/WBLCTrotTest.hpp>


template <typename T>
BackFlipCtrl<T>::BackFlipCtrl(
        const FloatingBaseModel<T>* robot, DataReader* data_reader):Controller<T>(robot),
    _data_reader(data_reader),
    _Kp(cheetah::num_act_joint),
    _Kd(cheetah::num_act_joint),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint),
    _jtorque(cheetah::num_act_joint),
    _end_time(1000.0),
    _dim_contact(0),
    _ctrl_start_time(0.)
{
    _sp = StateProvider<T>::getStateProvider();

    printf("[Body Control] Constructed\n");
}

template <typename T>
BackFlipCtrl<T>::~BackFlipCtrl(){
    delete _param_handler;
}

template <typename T>
void BackFlipCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

    _update_joint_command();

    for(size_t leg(0); leg<cheetah::num_leg; ++leg){
        for(size_t jidx(0); jidx<cheetah::num_leg_joint; ++jidx){
            ((LegControllerCommand<T>*)_cmd)[leg].tauFeedForward[jidx] 
                = _jtorque[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qDes[jidx] = 
                _des_jpos[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qdDes[jidx] = 
                _des_jvel[cheetah::num_leg_joint * leg + jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void BackFlipCtrl<T>::_update_joint_command(){
    _des_jpos.setZero();
    _des_jvel.setZero();
    _jtorque.setZero();
}

template <typename T>
void BackFlipCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
}

template <typename T>
void BackFlipCtrl<T>::LastVisit(){}

template <typename T>
bool BackFlipCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void BackFlipCtrl<T>::CtrlInitialization(const std::string & category_name){
    (void)category_name;
}

template <typename T>
void BackFlipCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);

    std::vector<T> tmp_vec;
    // Feedback Gain
    _param_handler->getVector<T>("Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _Kp[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>("Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _Kd[i] = tmp_vec[i];
    }
}

template class BackFlipCtrl<double>;
template class BackFlipCtrl<float>;
