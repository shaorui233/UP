/**
 * @file rt_gpio.c
 * @brief Hardware interface for GPIO pins on the ADL
 *
 * This driver was insipred by the driver provided by ADL, but has been rewritten to allow for faster updating.
 * In simulation, this code does nothing.
 *
 */
#include <stdio.h>
#include <unistd.h>

#ifndef SIMULATOR

#include <sys/io.h>

#endif

#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <rt/rt_gpio.h>

//registers from the default driver
typedef enum _SMBIO_REG {
  SMBIO_STATUS = 0,
  SMBIO_CTRL = 2,
  SMBIO_CMD = 3,
  SMBIO_TX_SLAVE_ADDR = 4,
  SMBIO_DATA_0 = 5,
  SMBIO_DATA_1 = 6,
  SMBIO_BLOCK_DATA = 7,
  SMBIO_PEC = 8,
  SMBIO_RX_SLAVE_ADDR = 9,
  SMBIO_RX_SLAVE_DATA = 0xA,
  SMBIO_AUX_STATUS = 0xC,
  SMBIO_AUX_CTRL = 0xD,
  SMBIO_SMLINK_PIN_CTRL = 0xE,
  SMBIO_SMBUS_PIN_CTRL = 0xF,
  SMBIO_SLAVE_STATUS = 0x10,
  SMBIO_SLAVE_CMD = 0x11,
  SMBIO_NOTIFY_DEV_ADDR = 0x14,
  SMBIO_NOTIFY_DLOW = 0x16,
  SMBIO_NOTIFY_DHIGH = 0x17,
} SMBIO_REG;

void update_gpio_port(int port, uint8_t data);

//base address of the GPIO controller
int baseport = 0;
int simulation_gpio = 0;
int init_error = 0;

//find baseport address using the lspci command and searching through the output 
//from original driver
/**
 * @brief Find the baseport of the system management bus
 *
 * Mostly from original driver provided by ADL.
 * @param baseport_prt Pointer to baseport
 * @return Success code
 */
int find_smbio_baseport(int *baseport_ptr) {
  int smbdev_found = 0, ret = -1;
  char *tmp_ptr, baseport_str[10], cmd_buf[128] = {0};
  FILE *fp;

  sprintf(cmd_buf, "lspci -v");
  fp = popen(cmd_buf, "r");
  if (fp) {
    while (fgets(cmd_buf, 128, fp)) {
      if ((tmp_ptr = strstr(cmd_buf, "SMBus"))) {
        smbdev_found = 1;
      }
      if (smbdev_found && (tmp_ptr = strstr(cmd_buf, "I/O ports at"))) {
        // Check for IO port number
        sscanf(tmp_ptr, "I/O ports at %s", baseport_str);
        *baseport_ptr = (int) strtol(baseport_str, NULL, 16);
        ret = 0;
        break;
      }
    }
    pclose(fp);
  }

  if (ret) {
    if (smbdev_found)
      printf("[ERROR: RT GPIO] SMBus controller found but IO ports are not enabled\n");
    else
      printf("[ERROR: RT GPIO] Cannot find SMBus controller on the list of PCI devices\n");
  }
  baseport = *baseport_ptr;
  printf("\t[RT GPIO] found baseport %x\n", baseport);

  if (ret) init_error = 1;
  return ret;

}

/**
 * @brief Write to the smbus until its clear
 *
 * Modified from the original version to have a shorter sleep time. 
 * This code looks like a bit of a hack...
 * @param baseport Baseport of smbus
 */
static void clear_smbus(int baseport) {
  if (init_error) return;
  char data, cnt = 0;
  data = inb(baseport);
  while (data && cnt++ < 50) {
    outb(data, baseport);
    //sleep was originally 100000 usecs, limiting output update frequency
    usleep(400);
    data = inb(baseport);
  }

}

/**
 * @brief Initialize GPIO
 *
 * Finds the baseport, sets up permissions, and writes to control registers
 */
void init_gpio(int sim) {
  if (sim) {
    simulation_gpio = 1;
    printf("[RT GPIO] Doing nothing.\n");
    return;
  }

  printf("[RT GPIO] Initializing for robot...\n");
  //if find_smbio_baseport fails, use backup default_baseport.
  int baseport = DEFAULT_BASEPORT;

  find_smbio_baseport(&baseport);

  //setup ioperm to allow inb/outb
  for (int i = 0; i <= 6; i++) {
    if (ioperm(baseport + i, 8, 1)) {
      init_error = 1;
      char error_msg[100];
      sprintf(error_msg, "[ERROR: RT GPIO] Failed to initialize GPIO %d - ioperm error: %s\n",
              baseport + i, strerror(errno));
      printf("%s", error_msg);
      return;
    }
  }

  //need to write 0x0 to ports 6, 7 to enable outputs
  printf("\t[RT GPIO] Setting control registers...\n");
  update_gpio_port(6, 0x0);
  update_gpio_port(7, 0x0);

  printf("[RT GPIO] Done!\n");
}

//buffer for outputs, flushed to output on calls to update_gpio()
uint16_t out_data = 0x0;

//modify output buffer
/**
 * @brief Modify GPIO output buffer
 * 
 * @param index Port to write to
 * @param value Value to write (0/1)
 */
void set_gpio(int index, int value) {
  if (simulation_gpio) return;
  if ((index > 15) || (index < 0)) {
    printf("[ERROR: RT GPIO] Index out of bounds.\n");
    return;
  }

  if (!value)
    out_data |= (0x1 << index);
  else
    out_data &= ~(0x1 << index);
}

/**
 * @brief Write buffer to GPIO pin for a specific port
 *
 * This function takes some time to return.
 *
 * @param port Port to update
 * @param data Data to write
 */
void update_gpio_port(int port, uint8_t data) {
  if (init_error) return;
  if (simulation_gpio) return;
  if (baseport == 0) {
    printf("[ERROR: RT GPIO] Bad baseport!\n");
    printf("[ERROR: RT GPIO] Using default baseport 0xf040.\n");
    baseport = 0xf040;
  }


  clear_smbus(baseport);  // Clear the SMBus before continue

  // Select the slave address with read/write bit clear
  outb(0x40, baseport + SMBIO_TX_SLAVE_ADDR);

  /* Select the port to write */
  outb(port, baseport + SMBIO_CMD);

  // Send data to port 0
  outb(data, baseport + SMBIO_DATA_0);

  //Trigger the SMBus transaction
  outb(0x48, baseport + SMBIO_CTRL);

}

/**
 * @brief Writes the buffer to pins for both ports
 *
 * This function takes some time to return.
 * Port 2 and 3 appear to be the 16 pins that are broken out to the connector.
 */
void update_gpio() {
  if (simulation_gpio) return;
  //the second set of 8 outputs is on a different "port"
  //to update all sixteen requires two messages to be sent
  update_gpio_port(2, out_data >> 8);
  update_gpio_port(3, out_data & 0xff);

}


/**
 * @brief Sets all pins LOW to disable the robot, logs the E-Stop, and prints a message
 *
 */
void estop() {

  for (int i = 0; i < 15; i++) set_gpio(i, 1);
  update_gpio();
  printf("ESTOP");
  printf("\n\n\n");
  printf("\x1b[31m---------------- \x1b[0m\n");
  printf("\x1b[31m     E-STOP \x1b[0m\n");
  printf("\x1b[31m---------------- \x1b[0m\n");
  printf("\n\n\n");

}

/** 
 * @brief Sets all pins HIGH to enable the robot.
 */
void enable_gpio() {


  printf("\n\n\n");
  printf("\x1b[33m---------------- \x1b[0m\n");
  printf("\x1b[33m     ENABLE! \x1b[0m\n");
  printf("\x1b[33m---------------- \x1b[0m\n");
  printf("\n\n\n");
  for (int i = 0; i < 15; i++) set_gpio(i, 0);
  update_gpio();

}
