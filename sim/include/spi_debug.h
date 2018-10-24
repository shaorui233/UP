#ifndef SPI_DEBUG_H
#define SPI_DEBUG_H

#include <QDialog>
#include <cheetahlcm/spi_command_t.hpp>
#include <cheetahlcm/spi_data_t.hpp>
#include <thread>
#include <iostream>
#include <mutex>

#include "lcm/lcm-cpp.hpp"

#define K_ZERO_BIT 0x04
#define K_TORQUE_BIT 0x02
#define K_ENABLE_BIT 0x01

using std::thread;
using std::mutex;
using std::cout;


namespace Ui {
class spi_debug;
}

class spi_debug : public QDialog
{
    Q_OBJECT

public:
    explicit spi_debug(QWidget *parent = 0);
    ~spi_debug();

private slots:
    void on_enable_button_clicked();
    void on_leg1_radio_button_clicked();
    void on_leg2_radio_button_clicked();
    void on_leg3_radio_button_clicked();
    void on_leg4_radio_button_clicked();
    void on_abad_radio_button_clicked();
    void on_hip_radio_button_clicked();
    void on_knee_radio_button_clicked();
    void on_zero_checkbox_stateChanged(int arg1);
    void on_low_torque_checkbox_stateChanged(int arg1);
    void on_enable_checkbox_stateChanged(int arg1);
    void on_q_des_slider_sliderMoved(int position);
    void on_qd_des_slider_sliderMoved(int position);
    void on_kp_slider_sliderMoved(int position);
    void on_kd_slider_sliderMoved(int position);
    void on_tau_ff_slider_sliderMoved(int position);

    void on_stop_button_clicked();
    void handle_spi_data(const lcm::ReceiveBuffer* rbuf,
                                                                  const std::string& chan,
                                                                  const cheetahlcm::spi_data_t* msg);

    void on_pd_everything_slider_sliderMoved(int position);

    void on_enable_all_checkbox_stateChanged(int arg1);

    void on_torque_all_checkbox_stateChanged(int arg1);

private:
    void set_safe_ui();
    void set_safe_message();
    void update_command_label();
    void update_data_label();
    void update_spi_driver_label();
    void update_lcm_label();
    void start_lcm();
    void stop_lcm();
    void run_lcm();
    bool valid_joint_selected();
    float map_slider(int slider, float max_val);
    float map_slider_positive(int slider, float max_val);

    Ui::spi_debug *ui;
    int leg = -1;
    int joint = 2;
    bool enabled = false;
    bool want_lcm_shutdown = false;

    cheetahlcm::spi_command_t spi_command;
    cheetahlcm::spi_data_t    spi_data;

    lcm::LCM lcm;
    mutex spi_data_mutex;
    std::thread* lcm_thread = nullptr;

    char command_label_text[1000];
    char data_label_text[1000];
    char spi_label_text[1000];
    char lcm_label_text[1000];
    int lcm_usecs = 10000;

    float q_max = 1.2f;
    float qd_max = 1.0f;
    float kp_max = 10.f;
    float kd_max = 3.f;
    float tau_max = 55.f;

    int lcm_send_count = 0;


};

#endif // SPI_DEBUG_H
