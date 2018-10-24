#include "spi_debug.h"
#include "ui_spi_debug.h"
#include <cstring>
#include <string>
#include <unistd.h>

spi_debug::spi_debug(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::spi_debug),
    lcm("udpm://239.255.76.67:7667?ttl=1")
{
    ui->setupUi(this);

    if(lcm.good())
    {
        cout<<"[SPI Debug] LCM Initialized succesfully\n";
    }
    else
    {
        cout<<"[ERROR: SPI Debug] LCM didn't initialize.\n";
    }
    lcm.subscribe("CHEETAH_spi_data", &spi_debug::handle_spi_data, this);
    memset(&spi_command,0,sizeof(spi_command));
    memset(&spi_data,0,sizeof(spi_data));
    ui->knee_radio_button->setChecked(false);
}

void spi_debug::handle_spi_data(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& chan,
                                           const cheetahlcm::spi_data_t* msg)
{
    spi_data = *msg;
}

spi_debug::~spi_debug()
{
    delete ui;

    // don't do anything else if the thread hasn't been created.
    if(lcm_thread == nullptr)
        return;

    // otherwise signal shutdown
    want_lcm_shutdown = true;
    // wait for LCM to quit
    lcm_thread->join();
    // delete thread object
    delete lcm_thread;
}

void spi_debug::set_safe_ui()
{
    ui->zero_checkbox->setChecked(false);
    ui->enable_checkbox->setChecked(false);
    ui->low_torque_checkbox->setChecked(false);
    ui->q_des_slider->setValue(50);
    ui->qd_des_slider->setValue(50);
    ui->kp_slider->setValue(0);
    ui->kd_slider->setValue(0);
    ui->tau_ff_slider->setValue(50);

}

void spi_debug::set_safe_message()
{
    for(int leg = 0; leg < 4; leg++)
    {
        spi_command.q_des_abad[leg]  = 0.f;
        spi_command.q_des_hip[leg]   = 0.f;
        spi_command.q_des_knee[leg]  = 0.f;
        spi_command.qd_des_abad[leg] = 0.f;
        spi_command.qd_des_hip[leg]  = 0.f;
        spi_command.qd_des_knee[leg] = 0.f;
        spi_command.kp_abad[leg]     = 0.f;
        spi_command.kp_hip[leg]      = 0.f;
        spi_command.kp_knee[leg]     = 0.f;
        spi_command.kd_abad[leg]     = 0.f;
        spi_command.kd_hip[leg]      = 0.f;
        spi_command.kd_knee[leg]     = 0.f;
        spi_command.tau_abad_ff[leg] = 0.f;
        spi_command.tau_hip_ff[leg]  = 0.f;
        spi_command.tau_knee_ff[leg] = 0.f;
        spi_command.flags[leg]       = 0;
    }
    update_command_label();

}

void spi_debug::update_command_label()
{
    char leg_text[200];

    memset(command_label_text,0,1000);
    sprintf(command_label_text,"Command:\n");

    for(int leg = 0; leg < 4; leg++)
    {
        sprintf(leg_text,"\tLeg %d\n"
                         "\t\tq_des: %.3f, %.3f, %.3f\n"
                         "\t\tqd_des: %.3f, %.3f, %.3f\n"
                         "\t\tkp: %.3f, %.3f, %.3f\n"
                         "\t\tkd: %.3f, %.3f, %.3f\n"
                         "\t\ttau: %.3f, %.3f, %.3f\n"
                         "\t\tflag: 0x%x\n",
                leg,
                spi_command.q_des_abad[leg], spi_command.q_des_hip[leg], spi_command.q_des_knee[leg],
                spi_command.qd_des_abad[leg], spi_command.qd_des_hip[leg], spi_command.qd_des_knee[leg],
                spi_command.kp_abad[leg], spi_command.kp_hip[leg], spi_command.kp_knee[leg],
                spi_command.kd_abad[leg], spi_command.kd_hip[leg], spi_command.kd_knee[leg],
                spi_command.tau_abad_ff[leg], spi_command.tau_hip_ff[leg], spi_command.tau_knee_ff[leg],
                spi_command.flags[leg]);
        strcat(command_label_text,leg_text);
    }

    ui->command_label->setText(QString(command_label_text));
}

void spi_debug::update_data_label()
{
    char leg_text[200];
    memset(data_label_text,0,1000);
    sprintf(data_label_text,"Data:\n");
    for(int leg = 0; leg < 4; leg++)
    {
        sprintf(leg_text,"\tLeg %d\n"
                "\t\tq: %.3f, %.3f, %.3f\n"
                "\t\tqd: %.3f, %.3f, %.3f\n"
                "\t\tflag: 0x%x\n",
                leg,
                spi_data.q_abad[leg], spi_data.q_hip[leg], spi_data.q_knee[leg],
                spi_data.qd_abad[leg], spi_data.qd_hip[leg], spi_data.qd_knee[leg],
                spi_data.flags[leg]);
        strcat(data_label_text,leg_text);
    }

    ui->data_label->setText(QString(data_label_text));

}

void spi_debug::update_spi_driver_label()
{
    sprintf(spi_label_text,"SPI Driver: 0x%hx", spi_data.spi_driver_status & 0xff);
    ui->spi_driver_label->setText(QString(spi_label_text));
}

void spi_debug::update_lcm_label()
{
    sprintf(lcm_label_text,"LCM:\n\tSent %d messages\n\tMost recently received message ID: %d", lcm_send_count,spi_data.spi_driver_status >> 16);
    ui->lcm_label->setText(QString(lcm_label_text));
}

void spi_debug::start_lcm()
{
    if(lcm_thread != nullptr)
    {
        cout<<"[SPI Debug] Error: start_lcm called when LCM hasn't been stopped\n";
        return;
    }

    cout<<"[SPI Debug] Starting SPI LCM thread.\n";
    want_lcm_shutdown = false;
    lcm_thread = new std::thread(&spi_debug::run_lcm, this);
}

void spi_debug::stop_lcm()
{
    if(lcm_thread == nullptr)
    {
        cout<<"[SPI Debug] Error: stop_lcm called when LCM hasn't been started\n";
        return;
    }

    want_lcm_shutdown = true;
    lcm_thread->join();
    delete lcm_thread;
    want_lcm_shutdown = false;
    lcm_thread = nullptr;
}

void spi_debug::run_lcm()
{
    cout<<"[SPI Debug] Hello! LCM thread: "<<lcm_usecs<<" us\n";

    for(;;)
    {
        if(want_lcm_shutdown)
        {
            cout<<"[SPI Debug] Goodbye.\n";
            break;
        }

        // send lcm
        lcm.publish("CHEETAH_spi_command_debug",&spi_command);
        // receive lcm
        lcm.handleTimeout(1);
        // update GUI
        update_data_label();
        update_spi_driver_label();
        update_lcm_label();
        // delay
        usleep(lcm_usecs);
        lcm_send_count++;
    }
}

bool spi_debug::valid_joint_selected()
{
    if(joint >= 0 && joint <= 2 && leg >= 0 && leg <= 3)
        return true;
    cout<<"[SPI Debug] Invalid joint selected try again.\n";

    return false;
}

float spi_debug::map_slider(int slider, float max_val)
{
    float v = (float)(slider - 50)/100.f;
    return v * max_val;
}

float spi_debug::map_slider_positive(int slider, float max_val)
{
    float v = (float)(slider)/100.f;
    return v * max_val;
}

void spi_debug::on_enable_button_clicked()
{
    if(enabled)
    {
        ui->enable_button->setText("Enable SPI Debug");
        stop_lcm();
        enabled = false;
    }
    else
    {
        set_safe_ui();
        set_safe_message();
        update_command_label();
        update_spi_driver_label();
        update_data_label();
        update_lcm_label();
        ui->enable_button->setText("Disable SPI Debug");
        start_lcm();
        enabled = true;
    }
}

void spi_debug::on_leg1_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    leg = 0;
}

void spi_debug::on_leg2_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    leg = 1;
}

void spi_debug::on_leg3_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    leg = 2;
}

void spi_debug::on_leg4_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    leg = 3;
}

void spi_debug::on_abad_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    joint = 0;
}

void spi_debug::on_hip_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    joint = 1;
}

void spi_debug::on_knee_radio_button_clicked()
{
    set_safe_ui();
    set_safe_message();
    joint = 2;
}

void spi_debug::on_zero_checkbox_stateChanged(int arg1)
{
    if(valid_joint_selected())
    {
        if(ui->zero_checkbox->isChecked())
            spi_command.flags[leg] |= K_ZERO_BIT;
        else
            spi_command.flags[leg] &= ~(K_ZERO_BIT);
    }


    update_command_label();
}

void spi_debug::on_low_torque_checkbox_stateChanged(int arg1)
{
    if(valid_joint_selected())
    {
        if(ui->low_torque_checkbox->isChecked())
            spi_command.flags[leg] |= K_TORQUE_BIT;
        else
            spi_command.flags[leg] &= ~(K_TORQUE_BIT);
    }

    update_command_label();
}

void spi_debug::on_enable_checkbox_stateChanged(int arg1)
{
    if(valid_joint_selected())
    {
        if(ui->enable_checkbox->isChecked())
            spi_command.flags[leg] |= K_ENABLE_BIT;
        else
            spi_command.flags[leg] &= ~(K_ENABLE_BIT);
    }

    update_command_label();
}

void spi_debug::on_q_des_slider_sliderMoved(int position)
{
    if(valid_joint_selected())
    {
        float v = map_slider(position,q_max);
        if(joint == 0)
            spi_command.q_des_abad[leg] = v;
        else if(joint == 1)
            spi_command.q_des_hip[leg] = v;
        else if(joint == 2)
            spi_command.q_des_knee[leg] = v;
        else
            cout<<"[SPI Debug] Error: on_q_des_slider_sliderMoved called with invalid joint\n";
    }

    update_command_label();
}

void spi_debug::on_qd_des_slider_sliderMoved(int position)
{
    if(valid_joint_selected())
    {
        float v = map_slider(position,qd_max);
        if(joint == 0)
            spi_command.qd_des_abad[leg] = v;
        else if(joint == 1)
            spi_command.qd_des_hip[leg] = v;
        else if(joint == 2)
            spi_command.qd_des_knee[leg] = v;
        else
            cout<<"[SPI Debug] Error: on_qd_des_slider_sliderMoved called with invalid joint\n";
    }

    update_command_label();
}

void spi_debug::on_kp_slider_sliderMoved(int position)
{
    if(valid_joint_selected())
    {
        float v = map_slider_positive(position,kp_max);
        if(joint == 0)
            spi_command.kp_abad[leg] = v;
        else if(joint == 1)
            spi_command.kp_hip[leg] = v;
        else if(joint == 2)
            spi_command.kp_knee[leg] = v;
        else
            cout<<"[SPI Debug] Error: on_kp_slider_sliderMoved called with invalid joint\n";
    }

    update_command_label();
}

void spi_debug::on_kd_slider_sliderMoved(int position)
{
    if(valid_joint_selected())
    {
        float v = map_slider_positive(position,kd_max);
        if(joint == 0)
            spi_command.kd_abad[leg] = v;
        else if(joint == 1)
            spi_command.kd_hip[leg] = v;
        else if(joint == 2)
            spi_command.kd_knee[leg] = v;
        else
            cout<<"[SPI Debug] Error: on_kd_slider_sliderMoved called with invalid joint\n";
    }

    update_command_label();
}

void spi_debug::on_tau_ff_slider_sliderMoved(int position)
{
    if(valid_joint_selected())
    {
        float v = map_slider(position,tau_max);
        if(joint == 0)
            spi_command.tau_abad_ff[leg] = v;
        else if(joint == 1)
            spi_command.tau_hip_ff[leg] = v;
        else if(joint == 2)
            spi_command.tau_knee_ff[leg] = v;
        else
            cout<<"[SPI Debug] Error: on_tau_ff_slider_sliderMoved called with invalid joint\n";
    }

    update_command_label();
}

void spi_debug::on_stop_button_clicked()
{
    set_safe_ui();
    set_safe_message();
}

void spi_debug::on_pd_everything_slider_sliderMoved(int position)
{
    float kp = 3*map_slider_positive(position,kp_max);
    float kd = 3*map_slider_positive(position,kd_max);

    for(int i = 0; i < 4; i++)
    {
        spi_command.kp_abad[i] = kp;
        spi_command.kp_hip[i]  = kp;
        spi_command.kp_knee[i] = kp;

        spi_command.kd_abad[i] = kd;
        spi_command.kd_hip[i]  = kd;
        spi_command.kd_knee[i] = kd;

        spi_command.qd_des_abad[i] = 0.f;
        spi_command.qd_des_hip[i] = 0.f;
        spi_command.qd_des_knee[i] = 0.f;

        spi_command.q_des_abad[i] = 0.f;
        spi_command.q_des_hip[i] = -.45f;
        spi_command.q_des_knee[i] = 1.f;

        spi_command.tau_abad_ff[i] = 0.f;
        spi_command.tau_hip_ff[i] = 0.f;
        spi_command.tau_knee_ff[i] = 0.f;

    }

    update_command_label();
}

void spi_debug::on_enable_all_checkbox_stateChanged(int arg1)
{
    for(int i = 0; i < 4; i++)
    {
        if(ui->enable_all_checkbox->isChecked())
            spi_command.flags[i] |= K_ENABLE_BIT;
        else
            spi_command.flags[i] &= ~(K_ENABLE_BIT);
    }
    update_command_label();
}

void spi_debug::on_torque_all_checkbox_stateChanged(int arg1)
{
    for(int i = 0; i < 4; i++)
    {
        if(ui->torque_all_checkbox->isChecked())
            spi_command.flags[i] |= K_TORQUE_BIT;
        else
            spi_command.flags[i] &= ~(K_TORQUE_BIT);
    }
    update_command_label();
}
