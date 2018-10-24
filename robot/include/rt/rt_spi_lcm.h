#ifndef _rt_spi_lcm
#define _rt_spi_lcm

#include <cheetahlcm_spi_data_t.h>
#include <cheetahlcm_spi_command_t.h>
#include <cheetahlcm_spi_torque_t.h>
#include <lcm/lcm.h>


void init_spi_lcm(lcm_t *main_lcm);

//void publish_spi_data(cheetahlcm_spi_data_t* data);
void get_spi_command_from_lcm(cheetahlcm_spi_command_t *cmd);

void publish_spi_torque(cheetahlcm_spi_torque_t *data);

#endif
