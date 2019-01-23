//#ifndef _rt_lcm
//#define _rt_lcm
//
//#include <stdint.h>
//#include <cheetahlcm_state_estimate_t.h>
//
//#define K_NUM_FLOATS_IN 20
//#define K_NUM_INTS_IN   10
//
//#define K_NUM_FLOATS_OUT 20
//#define K_NUM_INTS_OUT   10
//
//void init_lcm(int slow_mode, int use_etherCAT);
//
//void send_lcm(int mini_cheetah);
//
//void read_lcm();
//
//void increment_ecat_error();
//
//void increment_bad_imu_packet();
//
//int get_sim_loops();
//
//void publish_debug_stream(const char *name, double *values, int num_values);
//
//void publish_debug_values(double values[100], int num_values);
//
//
//void set_contact_estimate(double taud_filt[18], double f_foot[12], double contact_state[4],
//                          double contact_probability[4], double transition_state[4]);
//
//void set_state_estimate(double quat[4], double rpy[3], double p0[3], double v0[3],
//                        double pfoot[12], double stds[18], double gyro[3], double accelerometer[3],
//                        double contact_state[4], double raw_gyro[3]
//);
//
//void set_trot_info(
//        double mode,
//        double t_state,
//        double clock_time,
//        double trot_stance_time,
//        double phase,
//        double stance_times[4], double mode_request);
//
//void set_state_estimate_data(cheetahlcm_state_estimate_t *estimate);
//
//#endif
