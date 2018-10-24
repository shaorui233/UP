#ifndef SIM_LCM_HANDLER_H
#define SIM_LCM_HANDLER_H

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <cheetahlcm/sim_command_t.hpp>
#include <cheetahlcm/ecat_command_t.hpp>
#include <cheetahlcm/loop_counter_t.hpp>
#include <interfacelcm/gui_time_settings_t.hpp>
#include <interfacelcm/gui_environment_settings_t.hpp>
#include <cheetahlcm/spi_command_t.hpp>

#include "simulator_state.h"

// simulation LCM handler object.
// copied almost directly from old simulator

class sim_lcm_handler
{
public:
    sim_lcm_handler(SimulatorState* sim_state);
    void handleControlMessage(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan,
            const cheetahlcm::ecat_command_t* msg);

    void handleSPIMessage(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan,
            const cheetahlcm::spi_command_t* msg);

    void handleEnvironmentSettings(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan,
            const interfacelcm::gui_environment_settings_t* msg);

    void handleTimeSettings(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const interfacelcm::gui_time_settings_t* msg);


    void handleLoopCount(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan,
            const cheetahlcm::loop_counter_t* msg);
private:
    SimulatorState* sim_state;
};

#endif // SIM_LCM_HANDLER_H
