///** @file rt_ethercat.h
// *  @brief Interface to EtherCAT hardware
// */
//#ifndef _rt_ethercat
//#define _rt_ethercat
//
//#include <stdint.h>
//
//#include <stdint.h>
//
///**
// * @brief Union of float, interger, and unsigned integer for sending/receive over EtherCAT
// */
//typedef union {
//  float float_val;
//  int32_t int_val;
//  uint32_t uint_val;
//} ethercat32_t;
//
///**
// * @brief Enum of possible etherCAT data types.  Currently unused.
// */
//typedef enum {
//  FLOAT,
//  INT,
//  UINT
//} ethercat32_data_t;
//
//#define ETHERCAT_RECEIVE_x   0
//#define ETHERCAT_RECEIVE_y   1
//#define ETHERCAT_RECEIVE_z   2
//#define ETHERCAT_RECEIVE_dx  3
//#define ETHERCAT_RECEIVE_dy  4
//#define ETHERCAT_RECEIVE_dz  5
//#define ETHERCAT_RECEIVE_fx  6
//#define ETHERCAT_RECEIVE_fy  7
//#define ETHERCAT_RECEIVE_fz  8
//#define ETHERCAT_RECEIVE_q_abad  9
//#define ETHERCAT_RECEIVE_q_hip   10
//#define ETHERCAT_RECEIVE_q_knee  11
//#define ETHERCAT_RECEIVE_dq_abad 12
//#define ETHERCAT_RECEIVE_dq_hip  13
//#define ETHERCAT_RECEIVE_dq_knee 14
//#define ETHERCAT_RECEIVE_tau_abad    15
//#define ETHERCAT_RECEIVE_tau_hip 16
//#define ETHERCAT_RECEIVE_tau_knee    17
//#define ETHERCAT_RECEIVE_tau_des_abad    18
//#define ETHERCAT_RECEIVE_tau_des_hip 19
//#define ETHERCAT_RECEIVE_tau_des_knee    20
//#define ETHERCAT_RECEIVE_loop_count_ti   21
//#define ETHERCAT_RECEIVE_ethercat_count  22
//#define ETHERCAT_RECEIVE_microtime_ti    23
//#define ETHERCAT_RECEIVE_expansion25 24
//#define ETHERCAT_RECEIVE_expansion26 25
//#define ETHERCAT_RECEIVE_expansion27 26
//#define ETHERCAT_RECEIVE_expansion28 27
//#define ETHERCAT_RECEIVE_expansion29 28
//#define ETHERCAT_RECEIVE_expansion30 29
//#define ETHERCAT_RECEIVE_expansion31 30
//#define ETHERCAT_RECEIVE_expansion32 31
//#define ETHERCAT_RECEIVE_expansion33 32
//#define ETHERCAT_RECEIVE_expansion34 33
//#define ETHERCAT_RECEIVE_expansion35 34
//#define ETHERCAT_RECEIVE_expansion36 35
//#define ETHERCAT_RECEIVE_expansion37 36
//#define ETHERCAT_RECEIVE_expansion38 37
//#define ETHERCAT_RECEIVE_expansion39 38
//#define ETHERCAT_RECEIVE_expansion40 39
//
//#define ETHERCAT_SEND_x_des   0
//#define ETHERCAT_SEND_y_des   1
//#define ETHERCAT_SEND_z_des   2
//#define ETHERCAT_SEND_dx_des  3
//#define ETHERCAT_SEND_dy_des  4
//#define ETHERCAT_SEND_dz_des  5
//#define ETHERCAT_SEND_kpx 6
//#define ETHERCAT_SEND_kpy 7
//#define ETHERCAT_SEND_kpz 8
//#define ETHERCAT_SEND_kdx 9
//#define ETHERCAT_SEND_kdy 10
//#define ETHERCAT_SEND_kdz 11
//#define ETHERCAT_SEND_enable  12
//#define ETHERCAT_SEND_zero_joints 13
//#define ETHERCAT_SEND_fx_ff   14
//#define ETHERCAT_SEND_fy_ff   15
//#define ETHERCAT_SEND_fz_ff   16
//#define ETHERCAT_SEND_tau_abad_ff 17
//#define ETHERCAT_SEND_tau_hip_ff  18
//#define ETHERCAT_SEND_tau_knee_ff 19
//#define ETHERCAT_SEND_abad_zero_angle 20
//#define ETHERCAT_SEND_hip_zero_angle 21
//#define ETHERCAT_SEND_knee_zero_angle 22
//#define ETHERCAT_SEND_max_torque 23
//#define ETHERCAT_SEND_expansion25 24
//#define ETHERCAT_SEND_expansion26 25
//#define ETHERCAT_SEND_expansion27 26
//#define ETHERCAT_SEND_expansion28 27
//#define ETHERCAT_SEND_expansion29 28
//#define ETHERCAT_SEND_expansion30 29
//#define ETHERCAT_SEND_expansion31 30
//#define ETHERCAT_SEND_expansion32 31
//#define ETHERCAT_SEND_expansion33 32
//#define ETHERCAT_SEND_expansion34 33
//#define ETHERCAT_SEND_expansion35 34
//#define ETHERCAT_SEND_expansion36 35
//#define ETHERCAT_SEND_expansion37 36
//#define ETHERCAT_SEND_expansion38 37
//#define ETHERCAT_SEND_expansion39 38
//#define ETHERCAT_SEND_expansion40 39
//
//#define ETHERCAT_SLAVES_EXPECTED 4
//
//
///**
// * @brief Number of 32-bit etherCAT types to send
// */
//#define ETHERCAT_TO_SEND 40
///**
// * @brief Number of 32-bit etherCAT types to receive
// */
//#define ETHERCAT_TO_RECEIVE 40
//
//int init_ethercat(int sim);
//
//void send_data(int slave, int output, ethercat32_t data);
//
//ethercat32_t read_data(int slave, int output);
//
//void slave_send_receive();
//
//void send_data_as_float(int slave, int output, float data);
//
//void send_data_as_int(int slave, int output, int32_t data);
//
//void send_data_as_uint(int slave, int output, uint32_t data);
//
//float read_data_as_float(int slave, int output);
//
//int32_t read_data_as_int(int slave, int output);
//
//uint32_t read_data_as_uint(int slave, int output);
//
//ethercat32_t read_command(int slave, int output);
//
//float read_command_as_float(int slave, int output);
//
//int32_t read_command_as_int(int slave, int output);
//
//uint32_t read_command_as_uint(int slave, int output);
//
//void lock_ecat_sim_mutex();
//
//void unlock_ecat_sim_mutex();
//
//#endif
