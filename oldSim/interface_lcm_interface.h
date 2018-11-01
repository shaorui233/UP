#ifndef INTERFACE_LCM_INTERFACE_H
#define INTERFACE_LCM_INTERFACE_H

#include <iostream>
#include <mutex>
#include <thread>
#include <string>
#include <unistd.h>
#include <QString>
#include <QFileDialog>
#include <QSettings>
#include <interfacelcm/gui_contact_detection_settings_t.hpp>
#include <interfacelcm/gui_controller_balance_settings_t.hpp>
#include <interfacelcm/gui_controller_mpc_settings_t.hpp>
#include <interfacelcm/gui_controller_prmpc_settings_t.hpp>
#include <interfacelcm/gui_controller_swing_leg_settings_t.hpp>
#include <interfacelcm/gui_gait_settings_t.hpp>
#include <interfacelcm/gui_interesting_t.hpp>
#include <interfacelcm/gui_main_control_settings_t.hpp>
#include <interfacelcm/gui_mode_settings_t.hpp>
#include <interfacelcm/gui_state_estimator_settings_t.hpp>
#include <interfacelcm/rc_channels_t.hpp>
#include <interfacelcm/user_command_t.hpp>

#include "lcm/lcm-cpp.hpp"

using std::thread;
using std::mutex;
using std::cout;
using std::string;


// class to interface with controller over LCM

// lcm data is either double, float, or int16_t
// we need to keep track of these so we know how to look through arrays
// and decode float/doubles
enum lcm_datatype
{
    double_tt, float_tt, int16_t_t
};

// represents a single LCM value (so there are multiple for an array)
// the size field is the number of remaining entries (including the current)
// in the array.
// for a single number, size is 1 and data points to the variable
// name is the LCM variable name, with a _%d suffix for variables in an array
// type name is the LCM class/type thing.
struct lcm_variable
{
    int size;
    void* data;
    lcm_datatype type;
    char name[40];
    char type_name[40];

};

class interface_lcm_interface
{
public:
    interface_lcm_interface(int max_variables);
    // gets lcm variable at index
    lcm_variable* get_lcm_variable(int index);
    // get the total number of lcm variables
    int get_number_lcm_variables();
    // start LCM interface thread
    void start_interface_lcm();
    // stop LCM interface thread
    void stop_interface_lcm();
    // save LCM table variables to file
    void save_to_file();
    // load LCM table variables from file
    void load_from_file();
    // load LCM table variables from ../settings/default
    void load_defaults();

    void add_new_lcm_variable(int size, void* data, lcm_datatype type, const char* name, const char* type_name);

    ~interface_lcm_interface();

    // various LCM types
    interfacelcm::gui_contact_detection_settings_t       contact_detection_settings;
    interfacelcm::gui_controller_balance_settings_t      controller_balance_settings;
    interfacelcm::gui_controller_mpc_settings_t          controller_mpc_settings;
    interfacelcm::gui_controller_prmpc_settings_t        controller_prmpc_settings;
    interfacelcm::gui_controller_swing_leg_settings_t    controller_swing_leg_settings;
    interfacelcm::gui_gait_settings_t                    gait_settings;
    interfacelcm::gui_interesting_t                      interesting;
    interfacelcm::gui_main_control_settings_t            main_control_settings;
    //interfacelcm::gui_mode_settings_t                    mode_settings;
    interfacelcm::gui_state_estimator_settings_t         state_estimator_settings;
    //interfacelcm::rc_channels_t                          rc_channels;
    //interfacelcm::user_command_t                         user_command;
private:

    // function called by LCM thread
    void run_interface_lcm();
    // set to true to cause LCM thread to shutdown
    volatile bool want_lcm_shutdown = false;

    // function to add new lcm variable
    // it will show up in the GUI list automatically
    // and be sent over LCM automatically




    lcm::LCM lcm;
    int num_variables = 0;
    mutex interface_data_mutex;
    thread* interface_lcm_thread = nullptr;
    lcm_variable* variables;
    int max_variables = 0;
    // delay between LCM sends
    int lcm_usec_delay = 20000;

    // **** LCM TYPES



    // **** END LCM TYPES

};

#endif // INTERFACE_LCM_INTERFACE_H
