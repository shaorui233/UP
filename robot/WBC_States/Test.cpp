#include "Test.hpp"
#include <ParamHandler/ParamHandler.hpp>

template<typename T>
Test<T>::Test(FloatingBaseModel<T>* robot, const RobotType & robot_type):
    _b_first_visit(true),
    _robot(robot),
    _count(0),
    _waiting_count(5),
    _b_running(true)
{
    _robot_type = robot_type;
    _sp = StateProvider<T>::getStateProvider();
    _ParameterSetting();
    _state.q = DVec<T>::Zero(cheetah::num_act_joint);
    _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
    //printf("[Test] Constructed\n");
}

template <typename T>
Test<T>::~Test(){}

template <typename T>
void Test<T>::GetCommand(const Cheetah_Data<T>* data, 
        LegControllerCommand<T> * command, Cheetah_Extra_Data<T> * ext_data){
    for(size_t i(0); i < cheetah::num_act_joint; ++i){
        _state.q[i] = data->jpos[i];
        _state.qd[i] = data->jvel[i];
        
        _sp->_Q[i+6] = data->jpos[i];
        _sp->_Qdot[i+6] = data->jvel[i];
    }
    for(size_t i(0); i<4; ++i){
        _state.bodyOrientation[i] = data->body_ori[i];
    }
    _sp->_Q[cheetah::dim_config] = _state.bodyOrientation[0];
    _sp->_Q[0] = _state.bodyOrientation[1];
    _sp->_Q[1] = _state.bodyOrientation[2];
    _sp->_Q[2] = _state.bodyOrientation[3];


    _state.bodyPosition.setZero();
    _state.bodyVelocity.setZero();
    _state.bodyVelocity[0] = data->ang_vel[0];
    _state.bodyVelocity[1] = data->ang_vel[1];
    _state.bodyVelocity[2] = data->ang_vel[2];

    _robot->setState(_state);
    _robot->forwardKinematics();

    Vec3<T> ave_foot;
    Vec3<T> ave_foot_vel;
    ave_foot.setZero();
    ave_foot_vel.setZero();

    for(size_t i(0); i<_sp->_num_contact; ++i){
        ave_foot += (1./_sp->_num_contact) * _robot->_pGC[_sp->_contact_pt[i]];
        ave_foot_vel += (1./_sp->_num_contact) * _robot->_vGC[_sp->_contact_pt[i]];
    }
    _sp->_dir_command[0] = data->dir_command[0];
    _sp->_dir_command[1] = data->dir_command[1];

    for(size_t i(0); i<3; ++i)
        _sp->_ori_command[i] = data->ori_command[i];

    _state.bodyPosition = -ave_foot;
    _state.bodyPosition += _sp->_local_frame_global_pos;
   
    _state.bodyVelocity.tail(3) = -ave_foot_vel;

    _sp->_Q[3] = _state.bodyPosition[0];
    _sp->_Q[4] = _state.bodyPosition[1];
    _sp->_Q[5] = _state.bodyPosition[2];
 
    for(size_t i(0); i<6; ++i){
        _sp->_Qdot[i] = _state.bodyVelocity[i];
    }
  
    // Update with new body position
    _robot->setState(_state);
    _robot->forwardKinematics();

    // Update Mass, Gravity, Coriolis
    _robot->contactJacobians();
    _robot->massMatrix();
    _robot->gravityForce();
    _robot->coriolisForce();
    _SafetyCheck();
    if(_b_running){
        if(!_Initialization(data, command)){
            _UpdateTestOneStep();
            ComputeCommand(command);
        }
    }else{
        _SafeCommand(data, command);
    }
    _UpdateExtraData(ext_data);
    ++_count;
    // When there is sensed time
    _sp->_curr_time += cheetah::servo_rate;
}

template <typename T>
void Test<T>::_SafetyCheck(){
    if(fabs(_sp->_Q[0]) > _roll_limit){
        _b_running = false;
    }
    if(fabs(_sp->_Q[1]) > _pitch_limit){
        _b_running = false;
    }
}

template<typename T>
void Test<T>::_SafeCommand(const Cheetah_Data<T>* data, 
        LegControllerCommand<T> * command){

    for(size_t leg(0); leg<cheetah::num_leg; ++leg){
        for(size_t jidx(0); jidx<cheetah::num_leg_joint; ++jidx){
            command[leg].tauFeedForward[jidx] = 0.;
            command[leg].qDes[jidx] = data->jpos[3*leg + jidx];
            command[leg].qdDes[jidx] = 0.;
        }
    }
}


template <typename T>
bool Test<T>::_Initialization(
        const Cheetah_Data<T>* data, LegControllerCommand<T>* command){

    static bool test_initialized(false);
    if(!test_initialized) {
        _TestInitialization();
        test_initialized = true;
        printf("[Cheetah Test] Test initialization is done\n");
    }
    if(_count < _waiting_count){
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
void Test<T>::_ParameterSetting(){
    ParamHandler handler(CheetahConfigPath"INTERFACE_setup.yaml");
    handler.getValue<T>("roll_limit", _roll_limit);
    handler.getValue<T>("pitch_limit", _pitch_limit);
}

template <typename T>
void Test<T>::ComputeCommand(void* command){
  if(_b_first_visit){
    _state_list[_phase]->FirstVisit();
    _b_first_visit = false;
  }

  _state_list[_phase]->OneStep(command);

  if(_state_list[_phase]->EndOfPhase()){
    _state_list[_phase]->LastVisit();
    _phase = _NextPhase(_phase);
    _b_first_visit = true;
  }
}

template class Test<double>;
template class Test<float>;
