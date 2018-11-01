#include "sim_lcm_handler.h"
#include <stdlib.h>
#include <iostream>

using namespace std;

// copied from old simulator
// now takes a pointer to sim_state to access TI boards/sim settings/loop counts

sim_lcm_handler::sim_lcm_handler(SimulatorState* sim_state)
{
    //cout<<"[SIM LCM Handler] Hello.\n";

    this->sim_state = sim_state;
}


void sim_lcm_handler::handleControlMessage(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& chan,
                                           const cheetahlcm::ecat_command_t* msg)
{
    UNUSED(rbuf); UNUSED(chan);
    sim_state->control_mutex.lock();
    {
        for (int i = 0; i < 4; ++i)
        {
            sim_state->boards[i]->command.position_des[0] = msg->x_des[i];
            sim_state->boards[i]->command.position_des[1] = msg->y_des[i];
            sim_state->boards[i]->command.position_des[2] = msg->z_des[i];

            sim_state->boards[i]->command.velocity_des[0] = msg->dx_des[i];
            sim_state->boards[i]->command.velocity_des[1] = msg->dy_des[i];
            sim_state->boards[i]->command.velocity_des[2] = msg->dz_des[i];

            float mult = 1;
            sim_state->boards[i]->command.kp[0] = msg->kpx[i] * mult;
            sim_state->boards[i]->command.kp[1] = msg->kpy[i] * mult;
            sim_state->boards[i]->command.kp[2] = msg->kpz[i] * mult;

            sim_state->boards[i]->command.kd[0] = msg->kdx[i] * mult;
            sim_state->boards[i]->command.kd[1] = msg->kdy[i] * mult;
            sim_state->boards[i]->command.kd[2] = msg->kdz[i] * mult;


            sim_state->boards[i]->command.tau_ff[0] = msg->tau_abad_ff[i];
            sim_state->boards[i]->command.tau_ff[1] = msg->tau_hip_ff [i];
            sim_state->boards[i]->command.tau_ff[2] = msg->tau_knee_ff[i];

            sim_state->boards[i]->command.force_ff[0] = msg->fx_ff[i];
            sim_state->boards[i]->command.force_ff[1] = msg->fy_ff [i];
            sim_state->boards[i]->command.force_ff[2] = msg->fz_ff[i];

            sim_state->boards[i]->command.enable = msg->enable[i];
            sim_state->boards[i]->command.max_torque = msg->max_torque[i];

            sim_state->boards[i]->data_structure.ethercat_count_ti++;
            //printf("TI ECAT\n");
        }
    }
    sim_state->control_mutex.unlock();
}


void sim_lcm_handler::handleSPIMessage(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& chan,
                                           const cheetahlcm::spi_command_t* msg)
{
    sim_state->control_mutex.lock();
    memcpy((void*)(&sim_state->spi_cmd), msg, sizeof(cheetahlcm::spi_command_t));
    sim_state->control_mutex.unlock();
}

void sim_lcm_handler::handleEnvironmentSettings(const lcm::ReceiveBuffer* rbuf,
                                                const std::string& chan,
                                                const interfacelcm::gui_environment_settings_t* msg)
{
    UNUSED(rbuf); UNUSED(chan);
    sim_state->state_mutex.lock();
    {
        memcpy( &(sim_state->environment_settings), msg, sizeof(sim_state->environment_settings) );
    }
    sim_state->state_mutex.unlock();
}

void sim_lcm_handler::handleTimeSettings(const lcm::ReceiveBuffer* rbuf,
                                         const std::string& chan,
                                         const interfacelcm::gui_time_settings_t* msg)
{
    UNUSED(rbuf); UNUSED(chan);
    sim_state->state_mutex.lock();
    {
        memcpy( &(sim_state->time_settings), msg, sizeof(sim_state->time_settings)) ;
    }
    sim_state->state_mutex.unlock();
}

void sim_lcm_handler::handleLoopCount(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const cheetahlcm::loop_counter_t* msg)
{
    UNUSED(rbuf); UNUSED(chan);
    sim_state->loop_counter_mutex.lock();
    {
        sim_state->control_loop_count = msg->loop_count;
    }
    sim_state->loop_counter_mutex.unlock();
}
