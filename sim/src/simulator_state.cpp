#include "simulator_state.h"
#include <iostream>

// set up ti boards
SimulatorState::SimulatorState():
fr_ti (-1.f),
fl_ti (1.f),
rr_ti (-1.f),
rl_ti (1.f),
fr_spi (-1.f,0),
fl_spi (1.f,1),
rr_spi (-1.f,2),
rl_spi (1.f,3)
{
    boards[0] = &fr_ti;
    boards[1] = &fl_ti;
    boards[2] = &rr_ti;
    boards[3] = &rl_ti;

    spi_boards[0] = &fr_spi;
    spi_boards[1] = &fl_spi;
    spi_boards[2] = &rr_spi;
    spi_boards[3] = &rl_spi;

    for(int i = 0; i < 4; i++)
    {
        spi_boards[i]->cmd = &spi_cmd;
        spi_boards[i]->data = &spi_data;
    }

}

// set up link lengths for TI board kinematics function
void SimulatorState::set_link_lengths(sim_flt l1, sim_flt l2, sim_flt l3)
{
    for(int i = 0; i < NUM_LEGS; i++)
        boards[i]->set_link_lengths(l1,l2,l3);
}


// reset variables
// needs to run to set things to zero.
void SimulatorState::reset_state()
{
    control_loop_count = 0;
    state_mutex.lock();
    std::fill_n(cheetah_state.q, 12, 0);
    for(int i = 0 ; i < 2 ; i++)
    {
        cheetah_state.q[3*i+1] = -1.3;
        cheetah_state.q[3*i+2] =  2.3;
    }
    for(int i = 2 ; i < 4 ; i++)
    {
        cheetah_state.q[3*i+1] =  -1.3;
        cheetah_state.q[3*i+2] =  2.3;
    }
    cheetah_state.q[0] =  -0.2;
    cheetah_state.q[3] =   0.2;
    cheetah_state.q[6] =  -0.2;
    cheetah_state.q[9] =  0.2;

    std::fill_n(cheetah_state.qd, 12, 0);
    std::fill_n(environment_state.u, NUM_CONTACT_PTS*2, 0);

    for(int i = 0; i < NUM_LEGS; i++)
    {
        boards[i]->reset_ti_board_data();
        spi_boards[i]->reset_spine_board_data();
    }

    sim_flt xfb0[]  = { 1, 0, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0 };
    memcpy( cheetah_state.xfb, xfb0, sizeof(xfb0) );

    environment_settings.mu = 0.5;
    environment_settings.ground_k   = .5e6;
    environment_settings.ground_d   = 5e4;
    environment_settings.gravity[0] = 0;
    environment_settings.gravity[1] = 0;
    environment_settings.gravity[2] = -9.81;

    time_settings.time_step_high_level_control = 1e-3    ;
    time_settings.time_step_low_level_control  = 200e-6 ;
    time_settings.time_step_integration        = 100e-6  ;
    time_settings.graphics_frames_per_second   = 60      ;

    time_settings.slow_motion_factor = 1;
    time_settings.paused = 0;
    state_mutex.unlock();

    control_mutex.lock();
    for(int i = 0; i < NUM_LEGS; i++)
    {
        boards[i]->reset_ti_board_command();
        spi_boards[i]->reset_spine_board_command();
    }
    control_mutex.unlock();

    std::cout<<"[SIM] State reset!\n";
}
