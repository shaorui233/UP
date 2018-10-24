#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSurfaceFormat>
#include "simulator_interface.h"
#include "simgraphics_window.h"
#include "gamepad_monitor.h"
#include "interface_lcm_interface.h"
#include "spi_debug.h"
#include "robot_lcm_handler.h"

// main GUI window (QT)

namespace Ui {
class MainWindow;

}

// contains all data used for gui stuff
struct gui_settings_t
{
    double vx_max;
    double vy_max;
    double omega_max;
    double z_min;
    double z_max;
    double roll_max;
    double pitch_max;
    double yaw_max;
    double x_max;
    double y_max;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT // qt macro

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    // slots are like button callbacks
private slots:
    void on_sim_button_clicked();
    void on_robot_button_clicked();
    void on_start_sim_button_clicked();
    void update_sim_stats();
    void on_high_level_lineedit_editingFinished();
    void on_low_level_lineedit_editingFinished();
    void on_integration_lineedit_editingFinished();
    void on_slow_motion_lineedit_editingFinished();
    void on_ground_kp_lineedit_editingFinished();
    void on_ground_kd_lineedit_editingFinished();
    void on_pause_button_clicked();
    void on_home_button_clicked();
    void on_reset_button_clicked();
    void on_launch_graphics_button_clicked();
    void on_ground_mu_lineedit_editingFinished();
    void on_interface_lcm_button_clicked();
    void on_lcm_table_cellChanged(int row, int column);
    void on_save_lcm_button_clicked();
    void on_open_lcm_button_clicked();

    // force update of lcm table
    void force_update_lcm_table();

    void on_cheetah_3_v1_radio_clicked();

    void on_cheetah_3_v2_radio_clicked();

    void on_mini_cheetah_radio_clicked();


    void on_vx_lineEdit_editingFinished();

    void on_vy_lineEdit_editingFinished();

    void on_omega_lineEdit_editingFinished();

    void on_roll_lineEdit_editingFinished();

    void on_pitch_lineEdit_editingFinished();

    void on_yaw_lineEdit_editingFinished();

    void on_z_max_lineEdit_editingFinished();

    void on_z_min_lineEdit_editingFinished();

    void on_mini_spi_button_clicked();

    void on_volts_lineEdit_editingFinished();

    void on_graphics_lcm_button_clicked();



    void on_x_max_lineEdit_editingFinished();

    void on_y_max_lineEdit_editingFinished();

private:

    // enable/disable UI controls
    void set_sim_controls(bool enabled);
    void set_robot_controls(bool enabled);
    void set_lcm_table_controls(bool enabled);
    void set_main_settings_enabled(bool enabled);
    void set_gui_settings_enabled(bool enabled);

    void clear_robot_receive_labels();

    void reset_gui_settings(int mini_cheetah_defaults);

    void update_joystick();


    // set UI controls to defaults
    void load_default_sim_settings(bool use_mini_cheetah_defaults);

    // force current UI values to update
    // useful if they were modified when the simulator isn't running
    void force_update_all_ui();
    void force_update_gui_settings();



    // simulation interface is a slightly modified version of Pat's original simulator
    SimulatorInterface* sim_interface = nullptr;
    Ui::MainWindow *ui;

    // simulation statistics label
    char stat_string[200];
    char gfx_fps_string[200];
    char robot_lcm_log_string[200];
    char robot_graphics_string[200];
    char robot_monitor_string[200];
    bool paused = false;

    // interface lcm
    bool interface_lcm_running = false;

    bool robot_graphics_running = false;

    // surface format used to configure OpenGL context
    QSurfaceFormat format;
    // graphics window
    SimGraphicsWindow* sim_gfx = nullptr;

    interface_lcm_interface control_lcm;

    // uncomment for gamepad debugging
    GamepadMonitor gmon;

    gui_settings_t gui_settings;

    spi_debug spi_window;

    bool robot_type_selected = false;

    robot_lcm_handler* robot_lcm = nullptr;

    CheetahState robot_state;


};

#endif // MAINWINDOW_H
