#ifndef SIMULATORSTATE_H
#define SIMULATORSTATE_H

#include <mutex>

#include "common_types.h"
#include "ti_boardcontrol.h"
#include "spine_boardcontrol.h"
#include "interfacelcm/gui_time_settings_t.hpp"
#include "interfacelcm/gui_environment_settings_t.hpp"
#include "cheetahlcm/spi_command_t.hpp"
#include "cheetahlcm/spi_data_t.hpp"

// represents the state of the robot in simulation
// replaces the global variables previously used for state
// additionally includes TI board objects

struct CheetahState {
    sim_flt xfb[13];
    sim_flt q[12];
    sim_flt qd[12];
};

struct EnvironmentState {
    sim_flt u[NUM_CONTACT_PTS*2]; // Local contact deflection
};

class SimulatorState
{
public:
    SimulatorState();
    void reset_state();
    void set_link_lengths(sim_flt l1, sim_flt l2, sim_flt l3);
    std::mutex control_mutex, state_mutex, loop_counter_mutex;
    TI_BoardControl* boards[NUM_LEGS];
    spine_boardcontrol* spi_boards[NUM_LEGS];
    cheetahlcm::spi_command_t spi_cmd;
    cheetahlcm::spi_data_t spi_data;
    volatile s64 control_loop_count = 0;
    interfacelcm::gui_time_settings_t        time_settings;
    interfacelcm::gui_environment_settings_t environment_settings;
    CheetahState cheetah_state; // cheetah configuration
    EnvironmentState environment_state; // contact configuration
private:

    TI_BoardControl fr_ti, fl_ti, rr_ti, rl_ti; // simulated ti board
    spine_boardcontrol fr_spi, fl_spi, rr_spi, rl_spi; // simulated spi board


};

#endif // SIMULATORSTATE_H
