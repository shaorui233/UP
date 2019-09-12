#include "FrontJumpCtrl.hpp"


template <typename T>
FrontJumpCtrl<T>::FrontJumpCtrl(DataReader* data_reader,
                              float _dt) {

	_data_reader = data_reader;
	dt = _dt;
  _key_pt_step = ceil(dt*1000);

printf("dt: %f, step:%d\n", dt, _key_pt_step);
  _Kp.resize(12);
  _Kd.resize(12);
  _des_jpos.resize(12);
  _des_jvel.resize(12);
  _jtorque.resize(12);
  _Kp_joint.resize(3);
  _Kd_joint.resize(3);

}


template <typename T>
FrontJumpCtrl<T>::~FrontJumpCtrl() {}

template <typename T>
void FrontJumpCtrl<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) {
  _state_machine_time = _curr_time - _ctrl_start_time;

  _b_FrontJumpPreparation = b_preparation;
  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      command[leg].tauFeedForward[jidx] = _jtorque[3 * leg + jidx];
      command[leg].qDes[jidx] = _des_jpos[3 * leg + jidx] + 0 * _curr_time;
      command[leg].qdDes[jidx] = _des_jvel[3 * leg + jidx];
      command[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      command[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
    }
  }
}

template <typename T>
void FrontJumpCtrl<T>::_update_joint_command() {
  int pre_mode_duration(700);
  int leg_clearance_iteration(440);
  int leg_ramp_iteration(470);
  int tuck_iteration(600);
  int ramp_end_iteration(650);


   float tau_mult;

  _des_jpos.setZero();
  _des_jvel.setZero();
  _jtorque.setZero();

  if ( (pre_mode_count <  pre_mode_duration) || _b_FrontJumpPreparation) {  
    // move to the initial configuration to prepare for
    // FrontJumping
    if (pre_mode_count == 0) {
      printf("plan_timesteps: %d \n", _data_reader->plan_timesteps);
    }
    // printf("pre_mode_count: %d \n", pre_mode_count);
    
    pre_mode_count += _key_pt_step;
    current_iteration = 0;
    tau_mult = 0;
  } else {
    tau_mult = 1.2;
    // tau_mult = 1.;
  }

  if (current_iteration > _data_reader->plan_timesteps - 1) {
    current_iteration = _data_reader->plan_timesteps - 1;
  }

  float* current_step = _data_reader->get_plan_at_time(current_iteration);
  float* tau = current_step + tau_offset;

  Vec3<float> q_des_front;
  Vec3<float> q_des_rear;
  Vec3<float> qd_des_front;
  Vec3<float> qd_des_rear;
  Vec3<float> tau_front;
  Vec3<float> tau_rear;

  q_des_front << 0.0, current_step[3], current_step[4];
  q_des_rear << 0.0, current_step[5], current_step[6];
  qd_des_front << 0.0, current_step[10], current_step[11];
  qd_des_rear << 0.0, current_step[12], current_step[13];
  tau_front << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
  tau_rear << 0.0, tau_mult * tau[2] / 2.0, tau_mult * tau[3] / 2.0;

  //pretty_print(tau_front, std::cout, "tau front");
  //pretty_print(tau_rear, std::cout, "tau rear");
  float s(0.);

  if (current_iteration >= leg_clearance_iteration && current_iteration < tuck_iteration) {  // ramp to leg clearance for obstacle
    qd_des_front << 0.0, 0.0, 0.0;
    qd_des_rear << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
    tau_rear << 0.0, 0.0, 0.0;

    s = (float)(current_iteration - leg_clearance_iteration) /
        (leg_ramp_iteration - leg_clearance_iteration);

    if (s > 1) {
      s = 1;
    }

    Vec3<float> q_des_front_0;
    Vec3<float> q_des_rear_0;
    Vec3<float> q_des_front_f;
    Vec3<float> q_des_rear_f;

    current_step = _data_reader->get_plan_at_time(tuck_iteration);
    q_des_front_0 << 0.0, current_step[3], current_step[4];
    q_des_rear_0 << 0.0, current_step[5], current_step[6];

    current_step = _data_reader->get_plan_at_time(0);
    q_des_front_f << 0.0, -1.25, 2.65;
    q_des_rear_f << 0.0, -1.25, 2.65;


    q_des_front = (1 - s) * q_des_front_0 + s * q_des_front_f;
    q_des_rear = (1 - s) * q_des_rear_0 + s * q_des_rear_f;

  } else if (current_iteration >= tuck_iteration) { // ramp to landing configuration
    qd_des_front << 0.0, 0.0, 0.0;
    qd_des_rear << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
    tau_rear << 0.0, 0.0, 0.0;

    s = (float)(current_iteration - tuck_iteration) /
        (ramp_end_iteration - tuck_iteration);

    if (s > 1) {
      s = 1;
    }

    Vec3<float> q_des_front_0;
    Vec3<float> q_des_rear_0;
    Vec3<float> q_des_front_f;
    Vec3<float> q_des_rear_f;

    current_step = _data_reader->get_plan_at_time(tuck_iteration);
    q_des_front_0 << 0.0, current_step[3], current_step[4];
    q_des_rear_0 << 0.0, current_step[5], current_step[6];

    current_step = _data_reader->get_plan_at_time(0);
    // q_des_front_f << 0.0, current_step[3], current_step[4];
    // q_des_rear_f << 0.0, current_step[5], current_step[6];
    q_des_front_f << 0.0, -1.2, 2.4;
    q_des_rear_f << 0.0, -1.2, 2.4;

    q_des_front = (1 - s) * q_des_front_0 + s * q_des_front_f;
    q_des_rear = (1 - s) * q_des_rear_0 + s * q_des_rear_f;

  }

  // Abduction
  for (int i = 0; i < 12; i += 3) {
    _des_jpos[i] = 0.0;
    _des_jvel[i] = 0.0;
    _jtorque[i] = 0.0;
  }
  _des_jpos[0] = s * (-0.2);
  _des_jpos[3] = s * (0.2);
  _des_jpos[6] = s * (-0.2);
  _des_jpos[9] = s * (0.2);

  // Front Hip
  for (int i = 1; i < 6; i += 3) {
    _des_jpos[i] = q_des_front[1];
    _des_jvel[i] = qd_des_front[1];
    _jtorque[i] = tau_front[1];
  }

  // Front Knee
  for (int i = 2; i < 6; i += 3) {
    _des_jpos[i] = q_des_front[2];
    _des_jvel[i] = qd_des_front[2];
    _jtorque[i] = tau_front[2];
  }

  // Hind Hip
  for (int i = 7; i < 12; i += 3) {
    _des_jpos[i] = q_des_rear[1];
    _des_jvel[i] = qd_des_rear[1];
    _jtorque[i] = tau_rear[1];
  }

  // Hind Knee
  for (int i = 8; i < 12; i += 3) {
    _des_jpos[i] = q_des_rear[2];
    _des_jvel[i] = qd_des_rear[2];
    _jtorque[i] = tau_rear[2];
  }

  // Update rate 0.5kHz
  current_iteration += _key_pt_step;
}

template <typename T>
void FrontJumpCtrl<T>::FirstVisit(float _curr_time) {
  _ctrl_start_time = _curr_time;
  current_iteration = 0;
  pre_mode_count = 0;

}

template <typename T>
void FrontJumpCtrl<T>::LastVisit() {}

template <typename T>
bool FrontJumpCtrl<T>::EndOfPhase(LegControllerData<T>* data) {
  if (_state_machine_time > (_end_time - 2. * dt)) {
    return true;
  }

  for (int leg(0); leg < 4; ++leg) {
  		if(_state_machine_time>2.7 && data[leg].q[1] > 
          _q_knee_max && data[leg].qd[1] > _qdot_knee_max){
  	 		printf("Contact detected at leg [%d] => Switch to the landing phase !!! \n", leg); 
			  printf("state_machine_time: %lf \n",_state_machine_time); 
			  printf("Q-Knee: %lf \n",data[leg].q[1]);
  	 		printf("Qdot-Knee: %lf \n",data[leg].qd[1]);
  	 		return true;
  	 } 
  }

  return false;
}

template <typename T>
void FrontJumpCtrl<T>::SetParameter() {
  for (int i = 0;i < 12;i++) {
    _Kp[i] = 1000;
    _Kd[i] = 5.;
  }

  _Kp_joint = {20.0, 20.0, 20.0};
  _Kd_joint = {2.0, 2.0, 2.0};

}

template class FrontJumpCtrl<double>;
template class FrontJumpCtrl<float>;
