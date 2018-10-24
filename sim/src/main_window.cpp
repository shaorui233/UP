#include <stdio.h>
#include <iostream>
#include <QMessageBox>
#include <QTimer>
#include <QGamepad>

#include "main_window.h"
#include "ui_mainwindow.h"
#include "simulator_interface.h"



using std::cout;



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    control_lcm(500),
    gmon(this,&control_lcm)
{

    cout<<"[Cheetah Control] Opening main window...\n";
    ui->setupUi(this);
    ui->sim_stat_label->setText("Simulation time: N/A\nSimulation Speed: N/A\n");
    //set_sim_controls(false);
    //set_robot_controls(false);
    set_lcm_table_controls(false);
    set_gui_settings_enabled(false);
    set_main_settings_enabled(false);
    load_default_sim_settings(false);
    clear_robot_receive_labels();
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_sim_stats()));
    timer->start(16); // 16 ms is around 60 Hz, speed of most monitors
    //control_lcm.start_interface_lcm();

    QTimer* slow_timer = new QTimer(this);
    connect(slow_timer, SIGNAL(timeout()), this, SLOT(force_update_lcm_table()));
    slow_timer->start(100);

    //reset_gui_settings(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}



// ************************ SET ENABLED METHODS ********************************//
// enabled/disable most of the settings (used to disable controls before robot type chosen)
void MainWindow::set_main_settings_enabled(bool enabled)
{
    ui->launch_graphics_button->setEnabled(enabled);
    ui->interface_lcm_button->setEnabled(enabled);
    //ui->robot_button->setEnabled(enabled);
    //ui->sim_button->setEnabled(enabled);
}

// enable/disable table
void MainWindow::set_lcm_table_controls(bool enabled)
{
    ui->lcm_table->setEnabled(enabled);
    ui->open_lcm_button->setEnabled(enabled);
    ui->save_lcm_button->setEnabled(enabled);
}

// enable/disable UI
void MainWindow::set_robot_controls(bool enabled)
{

}

// enable/disable sim controls
void MainWindow::set_sim_controls(bool enabled)
{
    ui->start_sim_button->setEnabled(enabled);
    //    ui->cheetah_3_v1_radio->setEnabled(enabled);
    //    ui->cheetah_3_v2_radio->setEnabled(enabled);
    //    ui->mini_cheetah_radio->setEnabled(enabled);
    ui->pause_button->setEnabled(enabled);
    ui->home_button->setEnabled(enabled);
    ui->reset_button->setEnabled(enabled);

    ui->sim_stat_label->setEnabled(enabled);
    ui->sim_l_1->setEnabled(enabled);
    ui->sim_l_2->setEnabled(enabled);
    ui->sim_l_3->setEnabled(enabled);
    ui->sim_l_4->setEnabled(enabled);
    ui->sim_l_5->setEnabled(enabled);
    ui->sim_l_6->setEnabled(enabled);
    ui->sim_l_7->setEnabled(enabled);

    ui->high_level_lineedit->setEnabled(enabled);
    ui->low_level_lineedit->setEnabled(enabled);
    ui->integration_lineedit->setEnabled(enabled);
    ui->slow_motion_lineedit->setEnabled(enabled);
    ui->ground_kp_lineedit->setEnabled(enabled);
    ui->ground_kd_lineedit->setEnabled(enabled);
    ui->ground_mu_lineedit->setEnabled(enabled);
}

// enable/disable robot control settings
void MainWindow::set_gui_settings_enabled(bool enabled)
{
    ui->vx_lineEdit->setEnabled(enabled);
    ui->vy_lineEdit->setEnabled(enabled);
    ui->z_max_lineEdit->setEnabled(enabled);
    ui->z_min_lineEdit->setEnabled(enabled);
    ui->omega_lineEdit->setEnabled(enabled);
    ui->roll_lineEdit->setEnabled(enabled);
    ui->pitch_lineEdit->setEnabled(enabled);
    ui->yaw_lineEdit->setEnabled(enabled);
}


// ************************* FORCE UPDATE/RESET METHODS *******************************

// called at 60 Hz
// used to update simulation timing label and graphics
void MainWindow::update_sim_stats()
{
    update_joystick();

    // simulator
    if(sim_interface != nullptr)
        sprintf(stat_string,"Simulation time: %.3f\nSimulation Speed: %.3f\n",sim_interface->sim_time_statistic,sim_interface->ratio_statistic);
    else
        sprintf(stat_string,"Simulation time: N/A\nSimulation Speed: N/A\n");
    ui->sim_stat_label->setText(QString(stat_string));

    // graphics
    if(sim_gfx != nullptr)
    {
        sprintf(gfx_fps_string,"Graphics FPS: %.3f",sim_gfx->fps);
        if(sim_interface != nullptr && ui->simState_checkbox->isChecked())
        {
            sim_gfx->set_robot_state(sim_interface->get_cheetah_state());
        }

        if(robot_lcm != nullptr && !ui->simState_checkbox->isChecked())
        {
            for(int i = 0; i < 12; i++)
                robot_state.q[i] = robot_lcm->gfx.q[i];
            for(int i = 0; i < 13; i++)
                robot_state.xfb[i] = robot_lcm->gfx.xfb[i];

            sim_gfx->set_robot_state(&robot_state);
        }

    }
    else
        sprintf(gfx_fps_string,"Graphics FPS: N/A");

    ui->gfx_fps_label->setText(QString(gfx_fps_string));

    // robot labels
    if(robot_lcm != nullptr)
    {
        sprintf(robot_graphics_string,"Graphics: %d",robot_lcm->n_gfx);
        sprintf(robot_lcm_log_string,"LCM Log: %d", robot_lcm->n_error);
        sprintf(robot_monitor_string,"[RT %d]\nAvg: %d\nMin: %d\nMax: %d",robot_lcm->n_monitor, robot_lcm->monitor.avg,
                robot_lcm->monitor.min, robot_lcm->monitor.max);

        ui->graphics_label->setText(QString(robot_graphics_string));
        ui->lcm_log_label->setText(QString(robot_lcm_log_string));
        ui->monitor_label->setText(QString(robot_monitor_string));
    }





    //printf("from update_sim_stats, robot height is %.3f\n",sim_interface->get_cheetah_state()->xfb[6]);
}


// reset gui settings to reasonable defaults for the given robot
void MainWindow::reset_gui_settings(int mini_cheetah_defaults)
{
    if(mini_cheetah_defaults)
    {
        ui->vx_lineEdit->setText("0.4");
        ui->vy_lineEdit->setText("0.2");
        ui->omega_lineEdit->setText("1.0");
        ui->roll_lineEdit->setText("0.2");
        ui->pitch_lineEdit->setText("0.2");
        ui->yaw_lineEdit->setText("0.2");
        ui->z_max_lineEdit->setText("0.32");
        ui->z_min_lineEdit->setText("0.1");
        //ui->z_zero_lineEdit->setText("0.15");
        ui->x_max_lineEdit->setText(".1");
        ui->y_max_lineEdit->setText(".05");
    }
    else
    {
        ui->vx_lineEdit->setText("1.2");
        ui->vy_lineEdit->setText("0.5");
        ui->omega_lineEdit->setText("1.0");
        ui->roll_lineEdit->setText("0.2");
        ui->pitch_lineEdit->setText("0.2");
        ui->yaw_lineEdit->setText("0.2");
        ui->z_max_lineEdit->setText("0.5");
        ui->z_min_lineEdit->setText("0.3");
        //ui->z_zero_lineEdit->setText("0.4");
        ui->x_max_lineEdit->setText(".1");
        ui->y_max_lineEdit->setText(".05");
    }
    force_update_gui_settings();
}

// update values from joystick
void MainWindow::update_joystick()
{
    if(!interface_lcm_running) return;

    if(!ui->RC_checkbox->isChecked())
    {
        double vx = 0., vy = 0.f, omega = 0.f;
        double p_des[3] = {0,0, (gui_settings.z_max  + gui_settings.z_min) / 2.f};
        double rpy_des[3] = {0,0,0};

        // R2: xxx , Omega , vx , vy
        if(gmon.get_gamepad()->buttonR2())
        {
            vx = gmon.joystickmap(gmon.get_axis(1),-gui_settings.vx_max,gui_settings.vx_max);
            vy = gmon.joystickmap(gmon.get_axis(0),-gui_settings.vy_max,gui_settings.vy_max);
            omega = gmon.joystickmap(gmon.get_axis(2),-gui_settings.omega_max,gui_settings.omega_max);
        }
        else
        {
            // L2 :
            if(gmon.get_gamepad()->buttonL2())
            {
                rpy_des[1] = gmon.joystickmap(gmon.get_axis(3),-gui_settings.pitch_max,gui_settings.pitch_max);
                rpy_des[2] = gmon.joystickmap(gmon.get_axis(2),-gui_settings.yaw_max,gui_settings.yaw_max);
                p_des[2] = gmon.joystickmap(gmon.get_axis(1),gui_settings.z_min,gui_settings.z_max);
                rpy_des[0] = gmon.joystickmap(gmon.get_axis(0),-gui_settings.roll_max,gui_settings.roll_max);
            }
            // Nothing : pitch , yaw , x , y
            else
            {
                rpy_des[1] = gmon.joystickmap(gmon.get_axis(3),-gui_settings.pitch_max,gui_settings.pitch_max);
                rpy_des[2] = gmon.joystickmap(gmon.get_axis(2),-gui_settings.yaw_max,gui_settings.yaw_max);
                p_des[0] = gmon.joystickmap(gmon.get_axis(1),-gui_settings.x_max,gui_settings.x_max);
                p_des[1] = gmon.joystickmap(gmon.get_axis(0),-gui_settings.y_max,gui_settings.y_max);
            }
        }


        control_lcm.main_control_settings.v_des[0] = -vx;
        control_lcm.main_control_settings.v_des[1] = -vy;
        control_lcm.main_control_settings.omega_des[2] = -omega;
        for(int i = 0; i < 3; i++)
        {
            control_lcm.main_control_settings.p_des[i] = p_des[i];
            control_lcm.main_control_settings.rpy_des[i] = rpy_des[i];
        }
    }

}

// force UI simulator settings to be sent to simulator
void MainWindow::force_update_all_ui()
{
    cout<<"[Cheetah Control] Force update all UI controls\n";
    on_low_level_lineedit_editingFinished();
    on_high_level_lineedit_editingFinished();
    on_integration_lineedit_editingFinished();
    on_slow_motion_lineedit_editingFinished();
    on_ground_kd_lineedit_editingFinished();
    on_ground_kp_lineedit_editingFinished();
    on_ground_mu_lineedit_editingFinished();
}

// reset UI simulator settings to defaults (modified a bit to make mini cheetah more stable)
void MainWindow::load_default_sim_settings(bool use_mini_cheetah_defaults)
{
    if(use_mini_cheetah_defaults)
    {
        ui->high_level_lineedit->setText("1e-3");
        ui->low_level_lineedit->setText("2.5e-5");
        ui->integration_lineedit->setText("50e-6");
        ui->slow_motion_lineedit->setText("1");
        ui->ground_kp_lineedit->setText("5e4");
        ui->ground_kd_lineedit->setText("10e2");
        ui->ground_mu_lineedit->setText("0.8");
    }
    else
    {
        ui->high_level_lineedit->setText("1e-3");
        ui->low_level_lineedit->setText("200e-6");
        ui->integration_lineedit->setText("70e-6");
        ui->slow_motion_lineedit->setText("1");
        ui->ground_kp_lineedit->setText("5e5");
        ui->ground_kd_lineedit->setText("5e3");
        ui->ground_mu_lineedit->setText("0.8");
    }

    force_update_all_ui();
}

// update values stored in the table
// note that this is the opposite of the other force_update methods
// this loads values set from elsewhere and displays them in the GUI
void MainWindow::force_update_lcm_table()
{
    //bool was_running = interface_lcm_running;
    // hack to prevent edit callback from going
    //interface_lcm_running = false;
    //printf("force update\n");
    int k = control_lcm.get_number_lcm_variables();

    ui->lcm_table->setColumnCount(3);
    ui->lcm_table->setRowCount(k);

    for(int i = 0; i < k; i++)
    {
        lcm_variable* var = control_lcm.get_lcm_variable(i);

        QTableWidgetItem* cell = ui->lcm_table->item(i,0);
        if(!cell)
        {
            cell = new QTableWidgetItem;
            ui->lcm_table->setItem(i,0,cell);
        }
        cell->setText(QString::fromUtf8(var->type_name));

        cell = ui->lcm_table->item(i,1);
        if(!cell)
        {
            cell = new QTableWidgetItem;
            ui->lcm_table->setItem(i,1,cell);
        }
        cell->setText(QString::fromUtf8(var->name));

        cell = ui->lcm_table->item(i,2);
        if(!cell)
        {
            cell = new QTableWidgetItem;
            ui->lcm_table->setItem(i,2,cell);
        }

        if(var->type == double_tt)
            cell->setText(QString::number(*(double*)(var->data)));
        else if(var->type == float_tt)
            cell->setText(QString::number(*(float*)(var->data)));
        else
        {
            cout<<"[Main Window] LCM table got unsupported data type\n";
            cell->setText("???");
        }
    }
    //interface_lcm_running = was_running;
}

void MainWindow::clear_robot_receive_labels()
{
    ui->monitor_label->setText("Monitor: N/A");
    ui->graphics_label->setText("Graphics: N/A");
    ui->lcm_log_label->setText("LCM Log: N/A");
}





// ********************************* UI CALLBACKS **************************

// start up or shutdown simulator
void MainWindow::on_start_sim_button_clicked()
{
    if(sim_interface == nullptr)
    {
        // pick robot type
        cheetah_type robot;
        cout<<"\n\n\n*************FIRST-TIME SIMULATOR INIT*************\n";
        if(ui->mini_cheetah_radio->isChecked())
            robot = mini_cheetah;
        else if(ui->cheetah_3_v1_radio->isChecked())
            robot = cheetah_3_v1;
        else if(ui->cheetah_3_v2_radio->isChecked())
            robot = cheetah_3_v2;
        else
        {
            cout<<"FAIL - no robot type selected.\n";
            QMessageBox robot_error;
            robot_error.setText("ERROR: no robot selected.");
            robot_error.exec();
            return;
        }

        // create new simulator
        sim_interface = new SimulatorInterface(robot, ui->volts_lineEdit->text().toDouble());
        // update button
        ui->start_sim_button->setText("Kill Simulator");
        // force updates from UI changes made when simulator was off
        force_update_all_ui();
    }
    else
    {
        // shutdown simulator
        cout<<"\n\n\n******************KILL SIMULATOR*******************\n";
        ui->start_sim_button->setText("pls wait");
        // tells threads to exit
        sim_interface->want_shutdown = true;
        // I need a wait somewhere because LCM fails if restarted extremely rapidly (~200 ms)
        // this prevents user from restarting LCM too fast
        usleep(300000);
        // destructor waits for threads to join, just in case
        delete sim_interface;
        // reset to defaults
        sim_interface = nullptr;
        ui->start_sim_button->setText("Start Simulator!");
    }
}

void MainWindow::on_lcm_table_cellChanged(int row, int column)
{
    if(!interface_lcm_running)
        return;

    //cout<<"CHANGE!\n";

    if(column != 2)
    {
        cout<<"[Cheetah Control] LCM table name cell changed! Updating LCM table...\n";
        force_update_lcm_table();
        return;
    }

    lcm_variable* var = control_lcm.get_lcm_variable(row);
    if(var == nullptr)
    {
        cout<<"[Cheetah Control] LCM table edit error: get_lcm_variable returned NULL\n";
        QMessageBox lcm_table_error;
        lcm_table_error.setText("ERROR: LCM table edit failed (got NULL from get_lcm_variable)! The value has not been updated.");
        lcm_table_error.exec();
        return;
    }

    if(var->type == float_tt)
    {
        QTableWidgetItem* cell = ui->lcm_table->item(row,column);
        *(float*)(var->data) = cell->text().toFloat();
    }
    else if(var->type == double_tt)
    {
        QTableWidgetItem* cell = ui->lcm_table->item(row,column);
        *(double*)(var->data) = cell->text().toDouble();
    }
    else
    {
        cout<<"[Cheetah Control] LCM table edit error: unknown variable type\n";
        QMessageBox lcm_table_error;
        lcm_table_error.setText("ERROR: LCM table edit failed (unknown variable type)! The value has not been updated.");
        lcm_table_error.exec();
    }

}

void MainWindow::on_launch_graphics_button_clicked()
{
    if(sim_gfx == nullptr)
    {
        ui->launch_graphics_button->setText("Stop Graphics");
        format.setSamples(1);
        // very important to make stuff appear in the right order!!!
        format.setDepthBufferSize(24);
        sim_gfx = new SimGraphicsWindow(nullptr,!ui->mini_cheetah_radio->isChecked());
        sim_gfx->setFormat(format);
        sim_gfx->resize(1280,720);
        sim_gfx->show();
        sim_gfx->setAnimating(true);
    }
    else
    {
        ui->launch_graphics_button->setText("Launch Graphics");
        delete sim_gfx;
        sim_gfx = nullptr;
    }
}


void MainWindow::on_interface_lcm_button_clicked()
{
    if(interface_lcm_running)
    {
        ui->interface_lcm_button->setText("Start Interface LCM");
        control_lcm.stop_interface_lcm();
        interface_lcm_running = false;
        ui->lcm_table->clear();
        set_lcm_table_controls(false);
    }
    else
    {
        interface_lcm_running = false;
        ui->interface_lcm_button->setText("Stop Interface LCM");
        control_lcm.start_interface_lcm();
        control_lcm.load_defaults();
        force_update_lcm_table();
        interface_lcm_running = true;
        set_lcm_table_controls(true);
        ui->lcm_table->setColumnWidth(1,230);
        ui->lcm_table->setColumnWidth(0,130);
    }
    force_update_gui_settings();
}




void MainWindow::on_pause_button_clicked()
{
    if(sim_interface == nullptr)
        return;
    if(!paused)
    {
        paused = true;
        cout<<"[Cheetah Control] Simulator Pause\n";
        sim_interface->sim_state.control_mutex.lock();
        sim_interface->sim_state.time_settings.paused = paused;
        sim_interface->sim_state.control_mutex.unlock();
        ui->pause_button->setText("Un-Pause");
    }
    else
    {
        paused = false;
        cout<<"[Cheetah Control] Simulator Un-Pause\n";
        sim_interface->sim_state.control_mutex.lock();
        sim_interface->sim_state.time_settings.paused = paused;
        sim_interface->sim_state.control_mutex.unlock();
        ui->pause_button->setText("Pause");
    }

}

// modify xfb to put the cheetah at home again.
void MainWindow::on_home_button_clicked()
{
    if(sim_interface == nullptr)
        return;
    cout<<"[Cheetah Control] Come Home Cheetah! (this probably will end poorly)\n";
    sim_interface->sim_state.state_mutex.lock();
    sim_interface->sim_state.cheetah_state.xfb[0] = 1.f;
    for(int i = 1; i < 6; i++)
        sim_interface->sim_state.cheetah_state.xfb[i] = 0.f;
    sim_interface->sim_state.cheetah_state.xfb[6] = 0.3f; // so the robot isn't in the ground
    sim_interface->sim_state.state_mutex.unlock();
}

void MainWindow::on_high_level_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.time_settings.time_step_high_level_control = ui->high_level_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
    //ui->high_level_lineedit->text().toDouble()
}

void MainWindow::on_low_level_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.time_settings.time_step_low_level_control = ui->low_level_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}

void MainWindow::on_integration_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.time_settings.time_step_integration= ui->integration_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}

void MainWindow::on_slow_motion_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.time_settings.slow_motion_factor= ui->slow_motion_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}

void MainWindow::on_ground_kp_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.environment_settings.ground_k = ui->ground_kp_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}

void MainWindow::on_ground_kd_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.environment_settings.ground_d = ui->ground_kd_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}


void MainWindow::on_ground_mu_lineedit_editingFinished()
{
    if(sim_interface == nullptr)
        return;
    sim_interface->sim_state.control_mutex.lock();
    sim_interface->sim_state.environment_settings.mu = ui->ground_mu_lineedit->text().toDouble();
    sim_interface->sim_state.control_mutex.unlock();
}

// sim/robot radio button
void MainWindow::on_sim_button_clicked()
{
    set_sim_controls(true);
    set_robot_controls(false);
}

void MainWindow::on_robot_button_clicked()
{
    set_sim_controls(false);
    set_robot_controls(true);
}


void MainWindow::on_reset_button_clicked()
{


}

void MainWindow::on_save_lcm_button_clicked()
{
    control_lcm.save_to_file();

}

void MainWindow::on_open_lcm_button_clicked()
{
    control_lcm.load_from_file();
    force_update_lcm_table();
}

void MainWindow::on_cheetah_3_v1_radio_clicked()
{
    ui->volts_lineEdit->setText("60.0");
    on_volts_lineEdit_editingFinished();
    load_default_sim_settings(false);
    set_main_settings_enabled(true);
    set_gui_settings_enabled(true);
    if(!robot_type_selected)
        reset_gui_settings(0);
    robot_type_selected = true;
}

void MainWindow::on_cheetah_3_v2_radio_clicked()
{
    ui->volts_lineEdit->setText("60.0");
    on_volts_lineEdit_editingFinished();
    load_default_sim_settings(false);
    set_main_settings_enabled(true);
    set_gui_settings_enabled(true);
    if(!robot_type_selected)
        reset_gui_settings(0);
    robot_type_selected = true;
}

void MainWindow::on_mini_cheetah_radio_clicked()
{
    ui->volts_lineEdit->setText("24.0");
    on_volts_lineEdit_editingFinished();
    load_default_sim_settings(true);
    set_main_settings_enabled(true);
    set_gui_settings_enabled(true);
    if(!robot_type_selected)
        reset_gui_settings(1);
    robot_type_selected = true;
}


void MainWindow::force_update_gui_settings()
{
    gui_settings.omega_max = ui->omega_lineEdit->text().toDouble();
    gui_settings.pitch_max = ui->pitch_lineEdit->text().toDouble();
    gui_settings.roll_max =  ui->roll_lineEdit->text().toDouble();
    gui_settings.vx_max = ui->vx_lineEdit->text().toDouble();
    gui_settings.vy_max = ui->vy_lineEdit->text().toDouble();
    gui_settings.yaw_max = ui->yaw_lineEdit->text().toDouble();
    gui_settings.z_max = ui->z_max_lineEdit->text().toDouble();
    gui_settings.z_min = ui->z_min_lineEdit->text().toDouble();
    //gui_settings.z_zero = ui->z_zero_lineEdit->text().toDouble();
    gui_settings.x_max = ui->x_max_lineEdit->text().toDouble();
    gui_settings.y_max = ui->y_max_lineEdit->text().toDouble();
}

void MainWindow::on_vx_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_vy_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_omega_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_roll_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_pitch_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_yaw_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_z_max_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_z_min_lineEdit_editingFinished()
{
    force_update_gui_settings();
}



void MainWindow::on_x_max_lineEdit_editingFinished()
{
    force_update_gui_settings();
}

void MainWindow::on_y_max_lineEdit_editingFinished()
{
    force_update_gui_settings();
}


void MainWindow::on_mini_spi_button_clicked()
{
    spi_window.show();
}

void MainWindow::on_volts_lineEdit_editingFinished()
{
    if(sim_interface == nullptr) return;
    sim_interface->set_battery_voltage(ui->volts_lineEdit->text().toDouble());
}

void MainWindow::on_graphics_lcm_button_clicked()
{
    if(robot_graphics_running)
    {
        ui->graphics_lcm_button->setText("Start LCM Receive");
        clear_robot_receive_labels();
        delete robot_lcm;
        robot_lcm = nullptr;
        robot_graphics_running = false;
    }
    else
    {
        ui->graphics_lcm_button->setText("Stop LCM Receive");
        robot_lcm = new robot_lcm_handler();
        robot_graphics_running = true;
    }
}

