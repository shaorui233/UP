#include <cstdio>

#include "mip_sdk.h"
#include "mip_gx4_25.h"
#include "../LordImu.h"

#include <stdio.h>
#include <unistd.h>

#define DEFAULT_PACKET_TIMEOUT_MS  1000 // milliseconds, evidently
#define MIP_SDK_GX4_25_IMU_STANDARD_MODE  0x01


void mode_setup(mip_interface& interface) {

  (void)interface;
//
//  printf("----------------------------------------------------------------------\n");
//  printf("Requesting BASIC Status Report:\n");
//  printf("----------------------------------------------------------------------\n\n");
//
//  u32 IMU_MODEL_NUMBER = 6237;
//  //Request basic status report
//  while(mip_3dm_cmd_hw_specific_imu_device_status(&interface, IMU_MODEL_NUMBER, GX4_IMU_BASIC_STATUS_SEL, &imu_basic_field) != MIP_INTERFACE_OK){}
//
//  printf("Model Number: \t\t\t\t\t%04u\n", imu_basic_field.device_model);
//  printf("Status Selector: \t\t\t\t%s\n", imu_basic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
//  printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_basic_field.status_flags);
//  printf("System Millisecond Timer Count: \t\t%llu ms\n\n", imu_basic_field.system_timer_ms);
//
//  printf("Requesting DIAGNOSTIC Status Report:\n");
//
//  //Request diagnostic status report
//  while(mip_3dm_cmd_hw_specific_imu_device_status(&interface, GX4_IMU_MODEL_NUMBER, GX4_IMU_DIAGNOSTICS_STATUS_SEL, &imu_diagnostic_field) != MIP_INTERFACE_OK){}
//
//  printf("Model Number: \t\t\t\t\t%04u\n", imu_diagnostic_field.device_model);
//  printf("Status Selector: \t\t\t\t%s\n", imu_diagnostic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
//  printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_diagnostic_field.status_flags);
//  printf("System Millisecond Timer Count: \t\t%llu ms\n", imu_diagnostic_field.system_timer_ms);
//  printf("Magnetometer: \t\t\t\t\t%s\n", imu_diagnostic_field.has_mag == 1 ? "DETECTED" : "NOT-DETECTED");
//  printf("Pressure Sensor: \t\t\t\t%s\n", imu_diagnostic_field.has_pressure == 1 ? "DETECTED" : "NOT-DETECTED");
//  printf("Gyro Range Reported: \t\t\t\t%u deg/s\n", imu_diagnostic_field.gyro_range);
//  printf("Accel Range Reported: \t\t\t\t%u G\n", imu_diagnostic_field.accel_range);
//  printf("Magnetometer Range Reported: \t\t\t%f Gs\n", imu_diagnostic_field.mag_range);
//  printf("Pressure Range Reported: \t\t\t%f hPa\n", imu_diagnostic_field.pressure_range);
//  printf("Measured Internal Temperature: \t\t\t%f degrees C\n", imu_diagnostic_field.temp_degc);
//  printf("Last Temperature Measured: \t\t\t%u ms\n", imu_diagnostic_field.last_temp_read_ms);
//  printf("Bad Temperature Sensor Detected: \t\t%s\n", imu_diagnostic_field.temp_sensor_error == 1 ? "TRUE" : "FALSE");
//  printf("Number Received GPS Pulse-Per-Second Pulses: \t%u Pulses\n", imu_diagnostic_field.num_gps_pps_triggers);
//  printf("Time of Last GPS Pulse-Per-Second Pulse: \t%u ms\n", imu_diagnostic_field.last_gps_pps_trigger_ms);
//  printf("Data Streaming Enabled: \t\t\t%s\n", imu_diagnostic_field.stream_enabled == 1 ? "TRUE" : "FALSE");
//  printf("Number of Dropped Communication Packets: \t%u packets\n", imu_diagnostic_field.dropped_packets);
//  printf("Communications Port Bytes Written: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_written);
//  printf("Communications Port Bytes Read: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_read);
//  printf("Communications Port Write Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_write_overruns);
//  printf("Communications Port Read Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_read_overruns);
//
//  printf("\n\n");


}

int main(int argc, char** argv) {
  u32 com_port, baudrate;
  if(argc != 3) {
    printf("usage: imu-test com-port baudrate\n");
    return 1;
  }

  com_port = std::atoi(argv[1]);
  baudrate = std::atoi(argv[2]);

  LordImu imu;
  imu.init(com_port, baudrate);


//  mip_interface device_interface;
//
//  if(mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
//    printf("initialization error!\n");
//  }
//
//  mode_setup(device_interface);
  return 0;
}