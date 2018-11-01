#ifndef SIMULATORINTERFACE_H
#define SIMULATORINTERFACE_H

#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string>
#include <random>
#include <iostream>
#include <thread>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "simulator_math.h"
#include "simulator_state.h"
#include "TerrainSettings.h"
#include "sim_lcm_handler.h"

//#include "SingleStep/rt_nonfinite.h"
//#include "SingleStep/SingleStep.h"
//#include "SingleStep/rtwtypes.h"
//#include "SingleStep/SingleStep_types.h"
//#include "SingleStep/SingleStep_terminate.h"
//#include "SingleStep/SingleStep_initialize.h"
#include "lcm/lcm-cpp.hpp"

#include <cheetahlcm/imu_data_t.hpp>
#include <cheetahlcm/vectornav_data_t.hpp>
#include <simulatorlcm/sim_graphics_t.hpp>
#include <simulatorlcm/full_state_t.hpp>
#include <cheetahlcm/ecat_data_t.hpp>
#include <cheetahlcm/loop_counter_t.hpp>

#include <interfacelcm/gui_time_settings_t.hpp>
#include <interfacelcm/gui_environment_settings_t.hpp>
#include <cheetahlcm/spi_data_t.hpp>

#include "simulatorlcm/sim_torque_t.hpp"

using namespace std;

// class which wraps Pat's wrapper for SingleStep (spatial_v2 forward dynamics)

// select between different cheetah models
enum cheetah_type { cheetah_3_v1 = 0, cheetah_3_v2 = 1, mini_cheetah = 2 };

class SimulatorInterface
{
public:
    SimulatorInterface(cheetah_type robot,float v_bus);

    // terrain data file path
    string terrain_file_location;

    // simulation time
    volatile sim_flt sim_time_statistic = 0;

    // simulation ratio (larger = faster)
    volatile sim_flt ratio_statistic = 0;

    // if true, simulator threads shut down soon.
    volatile bool want_shutdown = false;
    CheetahState* get_cheetah_state();
    // state variables for simulated quantities
    SimulatorState sim_state;
    void set_battery_voltage(float v);
    ~SimulatorInterface();

private:
    float mini_cheetah_motor_model(float tau_des, float qd, float v, float gr, int leg, int joint);
    float cheetah_3_motor_model(float tau_des, float qd, float v, float gr, int leg, int joint);
    sim_flt sign(sim_flt x);

    void fast_reset_simulation();
    void full_reset_simulation();
    void reset_simulator_variables();
    void simulate_robot();
    void handle_lcm();
    static void getSysTime(timespec* ts);
    static sim_flt timeDiff(const timespec & t1, const timespec & t2);
    static sim_flt timeSince(const timespec & t1);

    static string cheetah_names[3];

    cheetah_type robot_type;
    bool first_iteration = true;
    std::random_device rd;
    std::mt19937 e2;
    std::uniform_real_distribution<> dist_accel;
    std::uniform_real_distribution<> dist_gyro;



    sim_flt tau_in[12];
    sim_flt p[NUM_CONTACT_PTS*3];
    sim_flt pd[NUM_CONTACT_PTS*3];
    sim_flt f[NUM_CONTACT_PTS*3];
    sim_flt last_normals[NUM_CONTACT_PTS*3];

    sim_flt passive_damping = 0;
    sim_flt dry_friction = 0;

    sim_flt next_ecat_time = 0, next_ti_time = 0;

    sim_flt t_sim = 0;
    s64 loopCount = 0;

    //struct0_T matlab_environment_struct;
    sim_flt gravity[3];

    interfacelcm::gui_time_settings_t current_time_settings;
    simulatorlcm::sim_graphics_t  graphics_lcm_data;
    simulatorlcm::full_state_t    state_lcm_data;
    simulatorlcm::sim_torque_t    sim_torque;

    cheetahlcm::ecat_data_t       ecat_data;
    cheetahlcm::imu_data_t        imu_data;
    cheetahlcm::vectornav_data_t  vectornav_data;
    // note that this is used for the simulator to tell the control code how far it has gone.
    cheetahlcm::loop_counter_t    loop_counter;

    TiBoardData recent_ti_board_data[4];
    cheetahlcm::spi_data_t recent_spi_data;
    timespec time_spec_start_sim, time_spec_last_ectat, time_spec_last_stdout , time_spec_last_graphics;

    lcm::LCM lcm;
    sim_lcm_handler lcm_handler;

    std::thread* lcm_thread;
    std::thread* sim_thread;

    s32 spi_id_counter = 0;
    float v_bus = 0;



};

#endif // SIMULATORINTERFACE_H
