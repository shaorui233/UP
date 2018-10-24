#include <lcm/lcm.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>

#include <rt/rt_interface_lcm.h>
#include <cheetahlcm_error_t.h>

#include <cheetahlcm_ecat_command_t.h>
#include <cheetahlcm_ecat_data_t.h>


// Streams from MATLAB
#include <cheetahlcm_state_estimate_t.h>
#include <cheetahlcm_contact_estimate_t.h>

volatile cheetahlcm_state_estimate_t state_estimate;
volatile cheetahlcm_contact_estimate_t contact_estimate;


#include <cheetahlcm_loop_counter_t.h>
#include <cheetahlcm_cheetah_graphics_t.h>

// Debug Streams
#include <cheetahlcm_user_debug_t.h>
#include <cheetahlcm_user_debug_stream_t.h>
#include <cheetahlcm_debug_trot_info_t.h>

volatile cheetahlcm_debug_trot_info_t trot_info;

// mini cheetah spi
#include <cheetahlcm_spi_command_t.h>
#include <cheetahlcm_spi_data_t.h>


#include <pthread.h>

#include <rt/rt_ethercat.h>
#include <rt/rt_lcm.h>
#include <rt/rt_spi_lcm.h>
#include <rt/rt_spi.h>

#define COPY_DATA(_aaa, _bbb, _nnn) { for(int _i = 0 ; _i<_nnn ; _i++){ _aaa[_i] = _bbb[_i]; } }


static lcm_t *lcm;
static pthread_mutex_t lcm_get_set_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile cheetahlcm_ecat_data_t debug_data;

volatile cheetahlcm_ecat_command_t adl_command;
volatile cheetahlcm_error_t error_data;
volatile cheetahlcm_loop_counter_t simulator_loop_counter;
volatile cheetahlcm_cheetah_graphics_t cheetah_graphics_info;

int slow_lcm = 0;
int lcm_pass_through_ethercat = 0;

void quatToR(const double quat[4], double R[3][3]) {
  double e0 = quat[0];
  double e1 = quat[1];
  double e2 = quat[2];
  double e3 = quat[3];

  R[0][0] = 1 - 2 * (e2 * e2 + e3 * e3);
  R[0][1] = 2 * (e1 * e2 - e0 * e3);
  R[0][2] = 2 * (e1 * e3 + e0 * e2);
  R[1][0] = 2 * (e1 * e2 + e0 * e3);
  R[1][1] = 1 - 2 * (e1 * e1 + e3 * e3);
  R[1][2] = 2 * (e2 * e3 - e0 * e1);
  R[2][0] = 2 * (e1 * e3 - e0 * e2);
  R[2][1] = 2 * (e2 * e3 + e0 * e1);
  R[2][2] = 1 - 2 * (e1 * e1 + e2 * e2);
}


void applyRotation(double a[3], double R[3][3], double b[3]) {
  b[0] = 0;
  b[1] = 0;
  b[2] = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      b[i] += R[i][j] * a[j];
    }
  }
}

void applyRotationTranspose(double a[3], double R[3][3], double b[3]) {
  b[0] = 0;
  b[1] = 0;
  b[2] = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      b[i] += R[j][i] * a[j];
    }
  }
}


double timeDiff(const struct timeval t1, const struct timeval t2);

struct timeval t_prev_grpahics;


static void
loop_counter_command_handler(const lcm_recv_buf_t *rbuf, const char *channel, const cheetahlcm_loop_counter_t *msg,
                             void *user) {
  simulator_loop_counter = *msg;
  //printf("Got Sim loop counter!\n");
}


void init_lcm(int slow_mode, int use_etherCAT) {
  if (slow_mode) slow_lcm = 1;
  if (use_etherCAT) lcm_pass_through_ethercat = 1;
  printf("[RT LCM] Initializing...\n");
  bzero((void *) &debug_data, sizeof debug_data);
  bzero((void *) &error_data, sizeof error_data);
  lcm = lcm_create("udpm://239.255.76.67:7667?ttl=255");
  if (!lcm) {
    printf("[ERROR: RT LCM] Unable to intialize lcm!\n");
    exit(-1);
  }
  printf("[RT LCM] Done!\n");
  cheetahlcm_loop_counter_t_subscribe(lcm, "SIMULATOR_loop_counter", &loop_counter_command_handler, NULL);

  init_interface_lcm(lcm);
  init_spi_lcm(lcm);


  //static int time_us = 0;
  gettimeofday(&t_prev_grpahics, NULL);
}


void increment_ecat_error() {
  error_data.ecat_dropped_packet++;
}

void increment_bad_imu_packet() {
  error_data.bad_imu_packet++;
}

void pass_through_ethercat_data() {
  lock_ecat_sim_mutex();
  for (int leg = 0; leg < 4; leg++) {
    //pass-through TI board debugging data
    debug_data.x[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_x);
    debug_data.y[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_y);
    debug_data.z[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_z);
    debug_data.dx[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dx);
    debug_data.dy[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dy);
    debug_data.dz[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dz);
    debug_data.fx[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_fx);
    debug_data.fy[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_fy);
    debug_data.fz[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_fz);
    debug_data.q_abad[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_q_abad);
    debug_data.q_hip[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_q_hip);
    debug_data.q_knee[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_q_knee);
    debug_data.dq_abad[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dq_abad);
    debug_data.dq_hip[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dq_hip);
    debug_data.dq_knee[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_dq_knee);

    debug_data.tau_abad[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_abad);
    debug_data.tau_hip[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_hip);
    debug_data.tau_knee[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_knee);

    debug_data.tau_des_abad[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_des_abad);
    debug_data.tau_des_hip[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_des_hip);
    debug_data.tau_des_knee[leg] = read_data_as_float(leg, ETHERCAT_RECEIVE_tau_des_knee);

    debug_data.loop_count_ti[leg] = read_data_as_uint(leg, ETHERCAT_RECEIVE_loop_count_ti);
    debug_data.ethercat_count_ti[leg] = read_data_as_uint(leg, ETHERCAT_RECEIVE_ethercat_count);
    debug_data.microtime_ti[leg] = read_data_as_uint(leg, ETHERCAT_RECEIVE_microtime_ti);

    //pass through ADL board commands to TI board
    adl_command.x_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_x_des);
    adl_command.y_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_y_des);
    adl_command.z_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_z_des);

    adl_command.dx_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_dx_des);
    adl_command.dy_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_dy_des);
    adl_command.dz_des[leg] = read_command_as_float(leg, ETHERCAT_SEND_dz_des);

    adl_command.kpx[leg] = read_command_as_float(leg, ETHERCAT_SEND_kpx);
    adl_command.kpy[leg] = read_command_as_float(leg, ETHERCAT_SEND_kpy);
    adl_command.kpz[leg] = read_command_as_float(leg, ETHERCAT_SEND_kpz);

    adl_command.kdx[leg] = read_command_as_float(leg, ETHERCAT_SEND_kdx);
    adl_command.kdy[leg] = read_command_as_float(leg, ETHERCAT_SEND_kdy);
    adl_command.kdz[leg] = read_command_as_float(leg, ETHERCAT_SEND_kdz);

    adl_command.enable[leg] = read_command_as_uint(leg, ETHERCAT_SEND_enable);
    adl_command.zero_joints[leg] = read_command_as_uint(leg, ETHERCAT_SEND_zero_joints);

    adl_command.fx_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_fx_ff);
    adl_command.fy_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_fy_ff);
    adl_command.fz_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_fz_ff);

    adl_command.tau_abad_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_tau_abad_ff);
    adl_command.tau_hip_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_tau_hip_ff);
    adl_command.tau_knee_ff[leg] = read_command_as_float(leg, ETHERCAT_SEND_tau_knee_ff);

    adl_command.abad_zero_angle[leg] = read_command_as_float(leg, ETHERCAT_SEND_abad_zero_angle);
    adl_command.hip_zero_angle[leg] = read_command_as_float(leg, ETHERCAT_SEND_hip_zero_angle);
    adl_command.knee_zero_angle[leg] = read_command_as_float(leg, ETHERCAT_SEND_knee_zero_angle);
    adl_command.max_torque[leg] = read_command_as_float(leg, ETHERCAT_SEND_max_torque);
  }
  unlock_ecat_sim_mutex();
}

double timeDiff(const struct timeval t1, const struct timeval t2) {
  return ((double) t2.tv_sec - t1.tv_sec) + (1.0e-6 * ((double) t2.tv_usec - t1.tv_usec));
}

void pass_through_single_leg_ethercat_data() {
  printf("[LCM Error] Single leg etherCAT passthrough not working now.\n");
}

void send_lcm(int mini_cheetah) {


  if (mini_cheetah) {
    //printf("ccc: 0x%x %.3f\n",get_spi_command(),get_spi_command()->q_des_abad[0]);

    cheetahlcm_spi_command_t_publish(lcm, "CHEETAH_spi_command", (cheetahlcm_spi_command_t *) get_spi_command());
    cheetahlcm_spi_data_t_publish(lcm, "CHEETAH_spi_data", (cheetahlcm_spi_data_t *) get_spi_data());
  } else {
#ifndef K_ETHERCAT_DISABLE
    if (lcm_pass_through_ethercat)
      pass_through_ethercat_data();
#endif
    cheetahlcm_ecat_data_t_publish(lcm, "CHEETAH_ecat_data", (cheetahlcm_ecat_data_t *) &debug_data);
    cheetahlcm_ecat_command_t_publish(lcm, "CHEETAH_ecat_command", (cheetahlcm_ecat_command_t *) &adl_command);
  }

  cheetahlcm_error_t_publish(lcm, "CHEETAH_error", (cheetahlcm_error_t *) &error_data);
  cheetahlcm_state_estimate_t_publish(lcm, "CHEETAH_state_estimate", (cheetahlcm_state_estimate_t *) &state_estimate);
  cheetahlcm_contact_estimate_t_publish(lcm, "CHEETAH_contact_estimate",
                                        (cheetahlcm_contact_estimate_t *) &contact_estimate);
  cheetahlcm_debug_trot_info_t_publish(lcm, "CHEETAH_debug_trot_info", (cheetahlcm_debug_trot_info_t *) &trot_info);


  //balanceControl_publish_data_lcm();

  struct timeval t_now;
  gettimeofday(&t_now, NULL);

  /* ----  BEGIN CHEETAH GRAPHICS ---- */
  if (timeDiff(t_prev_grpahics, t_now) > 1. / 30) {
    // I have no idea why this suddenly stopped working!!
    //printf("PUBLISHING CHEETAH GRPHICS!");
    memcpy((void *) (cheetah_graphics_info.xfb), (void *) state_estimate.quat, sizeof(state_estimate.quat));
    memcpy((void *) (cheetah_graphics_info.xfb + 4), (void *) state_estimate.p0, sizeof(state_estimate.p0));
    memcpy((void *) (cheetah_graphics_info.xfb + 7), (void *) state_estimate.gyro, sizeof(state_estimate.gyro));
    memcpy((void *) cheetah_graphics_info.rpy, (void *) state_estimate.rpy, sizeof(state_estimate.rpy));

//        for(int i = 0; i < 3; i++)
//        {
//            cheetah_graphics_info.xfb[i] = state_estimate.quat[i];
//            cheetah_graphics_info.xfb[i+4] = state_estimate.p0[i];
//            cheetah_graphics_info.xfb[i+7] = state_estimate.gyro[i];
//            cheetah_graphics_info.rpy[i] = state_estimate.rpy[i];
//        }
//        cheetah_graphics_info.xfb[3] = state_estimate.quat[3];

//        printf("lcm:\n");
//        for(int i = 0; i < 3; i++)
//            printf("%f, ", state_estimate.p0[i]);
//        printf("\n");
//        printf("size: %d\n", sizeof(state_estimate.p0));
//        for(int i = 0; i < 9; i++)
//            printf("%f, ", cheetah_graphics_info.xfb[i]);
//        printf("\n");
    if (mini_cheetah) {
      cheetah_graphics_info.q[0] = get_spi_data()->q_abad[0];
      cheetah_graphics_info.q[1] = get_spi_data()->q_hip[0];
      cheetah_graphics_info.q[2] = get_spi_data()->q_knee[0];
      cheetah_graphics_info.q[3] = get_spi_data()->q_abad[1];
      cheetah_graphics_info.q[4] = get_spi_data()->q_hip[1];
      cheetah_graphics_info.q[5] = get_spi_data()->q_knee[1];
      cheetah_graphics_info.q[6] = get_spi_data()->q_abad[2];
      cheetah_graphics_info.q[7] = get_spi_data()->q_hip[2];
      cheetah_graphics_info.q[8] = get_spi_data()->q_knee[2];
      cheetah_graphics_info.q[9] = get_spi_data()->q_abad[3];
      cheetah_graphics_info.q[10] = get_spi_data()->q_hip[3];
      cheetah_graphics_info.q[11] = get_spi_data()->q_knee[3];
    } else {
      cheetah_graphics_info.q[0] = debug_data.q_abad[0];
      cheetah_graphics_info.q[1] = debug_data.q_hip[0];
      cheetah_graphics_info.q[2] = debug_data.q_knee[0];
      cheetah_graphics_info.q[3] = debug_data.q_abad[1];
      cheetah_graphics_info.q[4] = debug_data.q_hip[1];
      cheetah_graphics_info.q[5] = debug_data.q_knee[1];
      cheetah_graphics_info.q[6] = debug_data.q_abad[2];
      cheetah_graphics_info.q[7] = debug_data.q_hip[2];
      cheetah_graphics_info.q[8] = debug_data.q_knee[2];
      cheetah_graphics_info.q[9] = debug_data.q_abad[3];
      cheetah_graphics_info.q[10] = debug_data.q_hip[3];
      cheetah_graphics_info.q[11] = debug_data.q_knee[3];
    }

    double R[3][3];
    quatToR((double *) cheetah_graphics_info.xfb, R);
    int kk = 0;

    for (int i = 0; i < 4; i++) {
      cheetah_graphics_info.p_foot[0][i] = state_estimate.pfoot[kk++];
      cheetah_graphics_info.p_foot[1][i] = state_estimate.pfoot[kk++];
      cheetah_graphics_info.p_foot[2][i] = state_estimate.pfoot[kk++];
      double f[3], f_global[3];
      f[0] = debug_data.fx[i];
      f[1] = debug_data.fy[i];
      f[2] = debug_data.fz[i];
      applyRotation(f, R, f_global);
      cheetah_graphics_info.f_foot[0][i] = -f_global[0];
      cheetah_graphics_info.f_foot[1][i] = -f_global[1];
      cheetah_graphics_info.f_foot[2][i] = -f_global[2];
    }
    double vb[3];
    applyRotationTranspose((double *) state_estimate.v0, R, vb);
    memcpy((void *) &cheetah_graphics_info.xfb[10], (void *) vb, sizeof(vb));

    //printf("ptr a: %lx, b: %lx\n", (void*)(cheetah_graphics_info.xfb+10), (void*)&cheetah_graphics_info.xfb[10]);




    cheetahlcm_cheetah_graphics_t_publish(lcm, "CHEETAH_cheetah_graphics",
                                          (cheetahlcm_cheetah_graphics_t *) &cheetah_graphics_info);
    t_prev_grpahics = t_now;
  }
  /* ----  END CHEETAH GRAPHICS ---- */
}

void read_lcm() {
  lcm_handle(lcm);
}

void publish_debug_stream(const char *name, double *values, int num_values) {
#ifndef WIRELESS
  cheetahlcm_user_debug_stream_t user_debug_stream;

  user_debug_stream.num_values = num_values;
  user_debug_stream.value = values;

  char stream_name[256];
  strcpy(stream_name, "DEBUG_");
  strcat(stream_name, name);
  cheetahlcm_user_debug_stream_t_publish(lcm, stream_name, &user_debug_stream);
#endif
}

void publish_debug_values(double values[100], int num_values) {
#ifndef WIRELESS
  cheetahlcm_user_debug_t user_debug;
  for (int i = 0; i < num_values; ++i) {
    user_debug.value[i] = values[i];
  }
  cheetahlcm_user_debug_t_publish(lcm, "CHEETAH_user_debug", &user_debug);
#endif

}

void set_contact_estimate(double taud_filt[18], double f_foot[12], double contact_state[4],
                          double contact_probability[4], double transition_state[4]) {
  COPY_DATA(contact_estimate.taud_filt, taud_filt, 18);
  COPY_DATA(contact_estimate.f_foot, f_foot, 12);
  COPY_DATA(contact_estimate.contact_state, contact_state, 4);
  COPY_DATA(contact_estimate.contact_probability, contact_probability, 4);
  COPY_DATA(contact_estimate.transition_state, transition_state, 4);
}


void set_state_estimate(double quat[4], double rpy[3], double p0[3], double v0[3],
                        double pfoot[12], double stds[18], double gyro[3], double accelerometer[3],
                        double contact_state[4], double raw_gyro[3]
) {

  COPY_DATA(state_estimate.quat, quat, 4);
  COPY_DATA(state_estimate.rpy, rpy, 3);
  COPY_DATA(state_estimate.p0, p0, 3);
  COPY_DATA(state_estimate.v0, v0, 3);
  COPY_DATA(state_estimate.pfoot, pfoot, 12);
  COPY_DATA(state_estimate.stds, stds, 18);
  COPY_DATA(state_estimate.gyro, gyro, 3);
  COPY_DATA(state_estimate.accelerometer, accelerometer, 3);

  COPY_DATA(state_estimate.contact_state, contact_state, 4);
  COPY_DATA(state_estimate.raw_gyro, raw_gyro, 3);

}

void set_state_estimate_data(cheetahlcm_state_estimate_t *estimate) {
  //printf("set_state_estimate: x %f\n", estimate->p0[0]);

  memcpy((void *) &state_estimate, estimate, sizeof(cheetahlcm_state_estimate_t));
}

void set_trot_info(
        double mode,
        double t_state,
        double clock_time,
        double trot_stance_time,
        double phase,
        double stance_times[4], double mode_request) {
  trot_info.mode = mode;
  trot_info.t_state = t_state;
  trot_info.clock_time = clock_time;
  trot_info.trot_stance_time = trot_stance_time;
  trot_info.phase = phase;
  COPY_DATA(trot_info.stance_times, stance_times, 4);
  trot_info.mode_request = mode_request;

}
