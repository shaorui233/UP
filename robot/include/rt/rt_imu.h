/**
 * @file rt_imu.h
 * @brief Hardware interface for serial IMU
 */
#ifndef _rt_serial
#define _rt_serial

void init_serial(int sim, int cheetah_3);

int get_matches();

int get_fails();

float *get_imu_data();

float get_imu_data_matlab(int index);

void set_blocking(int fd, int should_block, int port);

int set_interface_attribs(int fd, int speed, int parity, int port);

#endif
