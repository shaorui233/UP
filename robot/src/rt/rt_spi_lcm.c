#include <rt/rt_spi_lcm.h>

#include <cheetahlcm_spi_command_t.h>

volatile cheetahlcm_spi_command_t spi_command_debug;

#include <cheetahlcm_spi_data_t.h>
#include <cheetahlcm_spi_torque_t.h>

#include <pthread.h>

lcm_t *lcm_spi = NULL;
static pthread_mutex_t spi_lcm_mutex = PTHREAD_MUTEX_INITIALIZER;

static void
spi_lcm_command_handler(const lcm_recv_buf_t *rbuf, const char *channel, const cheetahlcm_spi_command_t *msg,
                        void *user) {
  pthread_mutex_lock(&spi_lcm_mutex);
  spi_command_debug = *msg;
  pthread_mutex_unlock(&spi_lcm_mutex);
  //printf("spi_debug recv cmd: %.3f\n", spi_command_debug.q_des_abad[0]);
}


void get_spi_command_from_lcm(cheetahlcm_spi_command_t *cmd) {
  pthread_mutex_lock(&spi_lcm_mutex);
  *cmd = spi_command_debug;
  //printf("spi_debug request cmd: %.3f\n", spi_command_debug.q_des_abad[0]);
  pthread_mutex_unlock(&spi_lcm_mutex);
}

// just adds the spi command handler to the main lcm
void init_spi_lcm(lcm_t *main_lcm) {
  printf("[RT SPI LCM] Initializing...\n");
  lcm_spi = main_lcm;

  cheetahlcm_spi_command_t_subscribe(lcm_spi, "CHEETAH_spi_command_debug", &spi_lcm_command_handler, NULL);
}

//void publish_spi_data(cheetahlcm_spi_data_t *data)
//{
//    cheetahlcm_spi_data_t_publish(lcm_spi,"CHEETAH_spi_data",data);
//}


void publish_spi_torque(cheetahlcm_spi_torque_t *data) {
  if (lcm_spi == NULL) return;
  cheetahlcm_spi_torque_t_publish(lcm_spi, "CHEETAH_spi_torque", data);
}
