///**
//
// * @file rt_ethercat.c
// * @brief Interface to EtherCAT hardware
// * Replaces EtherCAT with LCM when in simulation mode
// */
//
///**@brief EtherCAT errors are measured over this period of loop iterations */
//#define K_ETHERCAT_ERR_PERIOD 100
//
///**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
//#define K_ETHERCAT_ERR_MAX 20
//
//#define _GNU_SOURCE
//
//#include <stdio.h>
//#include <string.h>
//#include <inttypes.h>
//#include <stdlib.h>
//#include <time.h>
//#include <math.h>
//
//
//#include <rt/rt_gpio.h>
//#include <rt/rt_ethercat.h>
//#include <rt/rt_lcm.h>
//
//// SOEM from third-party
//#include <ethercat/ethercat.h>
//
//
///****************************
// *    SIMULATOR INCLUDES    *
// ***************************/
//#include <cheetahlcm_ecat_command_t.h>
//#include <cheetahlcm_ecat_data_t.h>
//#include <cheetahlcm_imu_data_t.h>
//
//volatile cheetahlcm_ecat_command_t ecat_command;
//volatile cheetahlcm_ecat_data_t ecat_data;
//pthread_mutex_t sim_data_mutex;
//
//
//static lcm_t *lcm_ecat;
////#endif
///*************************
// *        END SIM        *
// *************************/
//
//#define EC_TIMEOUTMON 500
//
//char IOmap[4096];
//OSAL_THREAD_HANDLE thread1;
//int expectedWKC;
//boolean needlf;
//volatile int wkc;
//boolean inOP;
//uint8 currentgroup = 0;
//
//int simulation_ec = 0;
//
///**
// * @brief Called when EtherCAT connection has failed.
// *
// * Does a GPIO E-Stop, writes to log file with reason/time, closes log file, and exits.
// * Does nothing when in simulation
// */
//void degraded_handler() {
//  //shut off gpio enables
//  estop();
//  printf("[ERROR: RT EtherCAT] Connection degraded!\n");
//  time_t current_time = time(NULL);
//  char *time_str = ctime(&current_time);
//  printf("[ERROR: ESTOP] EtherCAT became degraded at %s.\n", time_str);
//  printf("[SHUTDOWN] Stopping RT process.\n");
//  exit(0);
//}
//
////most of this function is from the EtherCAT example code
///**
// * @brief Starts an EtherCAT connection on a given interface
// *
// * Most of this function is from the SOEM example code
// * This function does nothing in simulation
// * @param ifname String for network interface
// */
//int run_ethercat(char *ifname) {
//  int i, oloop, iloop, chk;
//  needlf = FALSE;
//  inOP = FALSE;
//
//
//  /* initialise SOEM, bind socket to ifname */
//  if (ec_init(ifname)) {
//    printf("\t[RT EtherCAT] Initialization on device %s succeeded.\n", ifname);
//    /* find and auto-config slaves */
//
//    if (ec_config_init(FALSE) > 0) {
//      printf("\t[RT EtherCAT] %d slaves found and configured.\n", ec_slavecount);
//      if (ec_slavecount < ETHERCAT_SLAVES_EXPECTED) {
//        //char error_msg[200];
//        printf("[ERROR: RT EtherCAT] Expected %d legs, found %d.\n", ETHERCAT_SLAVES_EXPECTED, ec_slavecount);
//        //log_error(error_msg);
//      }
//
//      ec_config_map(&IOmap);
//      ec_configdc();
//
//      printf("\t[RT EtherCAT] Mapped slaves.\n");
//      /* wait for all slaves to reach SAFE_OP state */
//      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
//
//      oloop = ec_slave[0].Obytes;
//      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
//      if (oloop > 8) oloop = 8;
//      iloop = ec_slave[0].Ibytes;
//      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
//      if (iloop > 8) iloop = 8;
//
//      printf("\t[RT EtherCAT] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
//             ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
//
//      printf("\t[RT EtherCAT] Requesting operational state for all slaves...\n");
//      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
//      printf("\t[RT EtherCAT] Calculated workcounter %d\n", expectedWKC);
//      ec_slave[0].state = EC_STATE_OPERATIONAL;
//      /* send one valid process data to make outputs in slaves happy*/
//      ec_send_processdata();
//      ec_receive_processdata(EC_TIMEOUTRET);
//      /* request OP state for all slaves */
//      ec_writestate(0);
//      chk = 40;
//      /* wait for all slaves to reach OP state */
//      do {
//        ec_send_processdata();
//        ec_receive_processdata(EC_TIMEOUTRET);
//        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
//      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
//
//      if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
//        printf("\t[RT EtherCAT] Operational state reached for all slaves.\n");
//        inOP = TRUE;
//        return 1;
//
//      } else {
//        printf("[ERROR: RT EtherCAT] Not all slaves reached operational state.\n");
//        ec_readstate();
//
//        for (i = 1; i <= ec_slavecount; i++) {
//          if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
//            //char error_msg[100];
//
//            printf("[ERROR: RT EtherCAT] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
//                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
//            //log_error(error_msg);
//          }
//        }
//      }
//    } else {
//      printf("[ERROR: RT EtherCAT] No slaves found!\n");
//    }
//  } else {
//    printf("[ERROR: RT EtherCAT] No socket connection on %s - typically a permissions issue\n", ifname);
//  }
//  return 0;
//}
//
//int err_count = 0;
//int err_iteration_count = 0;
//
////thread to monitor the state of the etherCAT connection.
///**
// * @brief Function that monitors the state of the connection and attempts to keep all devices connected.
// *
// * Most of this code is from the EtherCAT example code.  This code should run in a separate thread as this function does not return.
// * This function does nothing in Simulation.
// */
//OSAL_THREAD_FUNC ecatcheck(void *ptr) {
//  //#ifndef SIMULATOR
//  printf("\t[RT EtherCAT] Starting ecatcheck loop\n");
//  int slave = 0;
//  while (1) {
//    //count errors
//    if (err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
//      err_iteration_count = 0;
//      err_count = 0;
//    }
//
//    if (err_count > K_ETHERCAT_ERR_MAX) {
//      //possibly shut down
//      //log_error("[EtherCAT Error] EtherCAT connection degraded.\n");
//      //log_error("[Simulink-Linux] Shutting down....\n");
//      degraded_handler();
//    }
//    err_iteration_count++;
//
//    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
//      if (needlf) {
//        needlf = FALSE;
//        printf("\n");
//      }
//      /* one ore more slaves are not responding */
//      ec_group[currentgroup].docheckstate = FALSE;
//      ec_readstate();
//      for (slave = 1; slave <= ec_slavecount; slave++) {
//        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
//          ec_group[currentgroup].docheckstate = TRUE;
//          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
//            printf("[ERROR: RT EtherCAT] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
//            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
//            ec_writestate(slave);
//            err_count++;
//            increment_ecat_error();
//          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
//            printf("[ERROR: RT EtherCAT] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
//            ec_slave[slave].state = EC_STATE_OPERATIONAL;
//            ec_writestate(slave);
//            err_count++;
//            //increment_ecat_error();
//          } else if (ec_slave[slave].state > 0) {
//            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
//              ec_slave[slave].islost = FALSE;
//              printf("[ERROR: RT EtherCAT] Slave %d reconfigured\n", slave);
//            }
//          } else if (!ec_slave[slave].islost) {
//            /* re-check state */
//            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
//            if (!ec_slave[slave].state) {
//              ec_slave[slave].islost = TRUE;
//              printf("[ERROR: RT EtherCAT] Slave %d lost\n", slave);
//              err_count++;
//              increment_ecat_error();
//              increment_ecat_error();
//            }
//          }
//        }
//        if (ec_slave[slave].islost) {
//          if (!ec_slave[slave].state) {
//            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
//              ec_slave[slave].islost = FALSE;
//              printf("[RT EtherCAT] Slave %d recovered\n", slave);
//            }
//          } else {
//            ec_slave[slave].islost = FALSE;
//            printf("[RT EtherCAT] Slave %d found\n", slave);
//          }
//        }
//      }
//      if (!ec_group[currentgroup].docheckstate)
//        printf("[RT EtherCAT] All slaves resumed OPERATIONAL.\n");
//    }
//    osal_usleep(50000);
//  }
//  //#endif
//}
//
//
///******************
// * SIMULATION LCM *
// *****************/
//
///**
// * @brief Handle LCM-EtherCAT for simulation
// *
// * In Simulation, LCM is used to provide EtherCAT data.
// * This function should run in its own thread and does not return
// * Not used in robot
// */
//static void *read_lcm_ecat(void *arg) {
//  printf("[RT EtherCAT] Simulator EtherCAT LCM read thread started.\n");
//  while (1)
//    lcm_handle(lcm_ecat);
//}
//
///**
// * @brief Handle incoming EtherCAT commands over LCM during Simulation
// *
// * This function is called by LCM whenever a new EtherCAT message arrives and copies the EtherCAT data into a buffer.
// *
// */
//static void
//command_handler_ecat(const lcm_recv_buf_t *rbuf, const char *channel, const cheetahlcm_ecat_data_t *msg, void *user) {
//  pthread_mutex_lock(&sim_data_mutex);
//  {
//    memcpy((void *) &ecat_data, msg, sizeof(ecat_data));
//  }
//  pthread_mutex_unlock(&sim_data_mutex);
//}
//
///******************
// *    END SIM     *
// *****************/
//
///**
// * @brief Initialize EtherCAT connection
// *
// * On the robot, this function starts the etherCAT monitoring thread and tries connecting to the legs every second for 100 seconds until a connection is made.
// *
// * In simulation, the EtherCAT LCM is initialized.
// */
//int init_ethercat(int sim) {
//  if (sim) {
//    simulation_ec = 1;
//    printf("[RT EtherCAT] Initialize for simulation...\n");
//    //in sim...
//    if (pthread_mutex_init(&sim_data_mutex, NULL) != 0) {
//      printf("[ERROR: RT EtherCAT] Failed to create simulation data mutex!\n");
//      return 1;
//    }
//    lcm_ecat = lcm_create("udpm://239.255.76.67:7667?ttl=255");
//    if (!lcm_ecat)
//      printf("[ERROR: RT EtherCAT] Unable to intialize lcm in rt_ethercat!\n");
//
//
//    cheetahlcm_ecat_data_t_subscribe(lcm_ecat, "SIMULATOR_ecat_data", &command_handler_ecat, NULL);
//    pthread_t lcm_read_thread;
//    int thread_rc = pthread_create(&lcm_read_thread, NULL, read_lcm_ecat, NULL);
//    if (thread_rc) printf("[ERROR: RT EtherCAT] Failed to create lcm read thread!\n");
//
//
//    printf("[RT EtherCAT] Done!\n");
//
//    for (int i = 0; i < 4; ++i) {
//      ecat_command.tau_abad_ff[i] = 0;
//      ecat_command.tau_hip_ff[i] = 0;
//      ecat_command.tau_knee_ff[i] = 0;
//      ecat_command.fx_ff[i] = 0;
//      ecat_command.fy_ff[i] = 0;
//      ecat_command.fz_ff[i] = 0;
//      ecat_command.max_torque[i] = 0;
//      ecat_command.enable[i] = 0;
//    }
//  } else {
//    printf("[RT EtherCAT] Initialize for robot...\n");
//    //initialize monitoring thread
//    osal_thread_create(&thread1, 128000, &ecatcheck, (void *) &ctime);
//
//    //try initialization until it succeeds
//    int i;
//    int rc;
//    for (i = 1; i < 100; i++) {
//      printf("\t[RT EtherCAT] Attempting to start EtherCAT, try %d of 100.\n", i);
//      rc = run_ethercat("p5p1");
//      if (rc) break;
//      osal_usleep(1000000);
//    }
//    if (rc) printf("\t[RT EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
//    else {
//      //char error_msg[100];
//      printf("[ERROR: RT EtherCAT] Failed to initialize EtherCAT after 100 tries. \n");
//      //log_error(error_msg);
//    }
//    printf("[RT EtherCAT] Done!\n");
//  }
//  return 0;
//}
//
//
//int wkc_err_count = 0;
//int wkc_err_iteration_count = 0;
//
////initiate etherCAT communication
///** @brief Send and receive data over EtherCAT
// *
// * In Simulation, send data over LCM
// * On the robt, verify the EtherCAT connection is still healthy, send data, receive data, and check for lost packets
// */
//void slave_send_receive() {
//  if (simulation_ec) {
//    // this is done in rt_lcm, we've been doing it twice.
//    //cheetahlcm_ecat_command_t_publish(lcm_ecat, "CHEETAH_ecat_command", (cheetahlcm_ecat_command_t *) &ecat_command);
//  } else {
//    //check connection
//    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
//      wkc_err_count = 0;
//      wkc_err_iteration_count = 0;
//    }
//    if (wkc_err_count > K_ETHERCAT_ERR_MAX) {
//      printf("[ERROR: RT EtherCAT] Error count too high!\n");
//      //program terminates in degraded handler.
//      degraded_handler();
//    }
//
//    //send
//    ec_send_processdata();
//    //receive
//    wkc = ec_receive_processdata(EC_TIMEOUTRET);
//
//    //check for dropped packet
//    if (wkc < expectedWKC) {
//      printf("\x1b[ERROR: RT EtherCAT] Dropped packet (Bad WKC!)\x1b[0m\n");
//      wkc_err_count++;
//      increment_ecat_error();
//    }
//    wkc_err_iteration_count++;
//  }
//}
//
///** @brief Modify EtherCAT output buffer.
// *
// * @param slave Leg number to send data to
// * @param output Output number to change
// * @param data Data to send
// */
//void send_data(int slave, int output, ethercat32_t data) {
//  if (slave > ETHERCAT_SLAVES_EXPECTED - 1) return;
//  if (simulation_ec) {
//    //pthread_mutex_lock(&sim_data_mutex);
//    ethercat32_t *data_ptr = ((ethercat32_t *) &ecat_command) + sizeof(ethercat32_t) * output + slave;
//    *data_ptr = data;
//    //pthread_mutex_unlock(&sim_data_mutex);
//  } else {
//    ethercat32_t *data_ptr = (ethercat32_t *) (ec_slave[slave + 1].outputs + output * sizeof(ethercat32_t));
//    *data_ptr = data;
//  }
//}
//
///** @brief Set EtherCAT output buffer to a float
// *
// * @param slave Leg to send data to
// * @param output Output number to change
// * @param data Float to set
// */
//void send_data_as_float(int slave, int output, float data) {
//  if (isnan(data)) {
//    printf("[ERROR: RT EtherCAT] Got NaN for slave %d, output %d!\n", slave, output);
//    data = -1;
//  }
//
//  ethercat32_t value;
//  value.float_val = data;
//  send_data(slave, output, value);
//}
//
//
///** @brief Set EtherCAT output buffer to an int
// *
// * @param slave Leg to send data to
// * @param output Output number to change
// * @param data Int to set
// */
//void send_data_as_int(int slave, int output, int32_t data) {
//  ethercat32_t value;
//  value.int_val = data;
//  send_data(slave, output, value);
//}
//
///** @brief Set EtherCAT output buffer to a uint
// *
// * @param slave Leg to send data to
// * @param output Output number to change
// * @param data Uint to set
// */
//void send_data_as_uint(int slave, int output, uint32_t data) {
//  ethercat32_t value;
//  value.uint_val = data;
//  send_data(slave, output, value);
//}
//
///** @brief Read data from EtherCAT buffer
// *
// * @param slave Leg to read data from
// * @param output Output number to read
// * @return Data from buffer
// */
//ethercat32_t read_data(int slave, int output) {
//  if (slave > ETHERCAT_SLAVES_EXPECTED - 1) return (ethercat32_t) 0;
//  ethercat32_t value;
//  if (simulation_ec) {
//    //pthread_mutex_lock(&sim_data_mutex);
//    ethercat32_t *ptr = ((ethercat32_t *) &ecat_data) + output * sizeof(ethercat32_t) + slave;
//    value = *ptr;
//    //pthread_mutex_unlock(&sim_data_mutex);
//  } else {
//    ethercat32_t *int_ptr = (ethercat32_t *) (ec_slave[slave + 1].inputs + output * sizeof(ethercat32_t) + 0);
//    value = *int_ptr;
//  }
//  return value;
//}
//
//void lock_ecat_sim_mutex() {
//  pthread_mutex_lock(&sim_data_mutex);
//}
//
//void unlock_ecat_sim_mutex() {
//  pthread_mutex_unlock(&sim_data_mutex);
//}
//
///** @brief Read data from EtherCAT receive buffer as float
// *
// * @param slave Leg to read data from
// * @param output Output number to read
// * @return Data from buffer
// */
//float read_data_as_float(int slave, int output) {
//  ethercat32_t value = read_data(slave, output);
//  return value.float_val;
//}
//
///** @brief Read data from EtherCAT receive buffer as int
// *
// * @param slave Leg to read data from
// * @param output Output number to read
// * @return Data from buffer
// */
//int32_t read_data_as_int(int slave, int output) {
//  ethercat32_t value = read_data(slave, output);
//  return value.int_val;
//}
//
///** @brief Read data from EtherCAT receive buffer as uint
// *
// * @param slave Leg to read data from
// * @param output Output number to read
// * @return Data from buffer
// */
//uint32_t read_data_as_uint(int slave, int output) {
//  ethercat32_t value = read_data(slave, output);
//  return value.uint_val;
//}
//
//
///** @brief Read data from EtherCAT send buffer
// *
// * @param slave Leg to read from
// * @param output Output to read from
// * @return Data from buffer
// */
//ethercat32_t read_command(int slave, int output) {
//  if (slave > ETHERCAT_SLAVES_EXPECTED - 1) return (ethercat32_t) 0;
//  if (simulation_ec) {
//    //pthread_mutex_lock(&sim_data_mutex);
//    ethercat32_t *data_ptr = ((ethercat32_t *) &ecat_command) + sizeof(ethercat32_t) * output + slave;
//    ethercat32_t out = *data_ptr;
//    //pthread_mutex_unlock(&sim_data_mutex);
//    return out;
//  } else {
//    ethercat32_t value;
//    ethercat32_t *ecat_ptr = (ethercat32_t *) (ec_slave[slave + 1].outputs + output * sizeof(ethercat32_t) + 0);
//    value = *ecat_ptr;
//    return value;
//  }
//}
//
///** @brief Read data from EtherCAT send buffer as float
// *
// * @param slave Leg to read from
// * @param output Output to read from
// * @return Data from buffer
// */
//float read_command_as_float(int slave, int output) {
//  ethercat32_t value = read_command(slave, output);
//  return value.float_val;
//}
//
///** @brief Read data from EtherCAT send buffer as int
// *
// * @param slave Leg to read from
// * @param output Output to read from
// * @return Data from buffer
// */
//int32_t read_command_as_int(int slave, int output) {
//  ethercat32_t value = read_command(slave, output);
//  return value.int_val;
//}
//
///** @brief Read data from EtherCAT send buffer as uint
// *
// * @param slave Leg to read from
// * @param output Output to read from
// * @return Data from buffer
// */
//uint32_t read_command_as_uint(int slave, int output) {
//  ethercat32_t value = read_command(slave, output);
//  return value.uint_val;
//}
//
