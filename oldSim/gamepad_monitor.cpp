#include "gamepad_monitor.h"
#include <QtGamepad/QGamepad>
#include <iostream>
#include <QDebug>



using std::cout;

// class to get joystick data for control_lcm
GamepadMonitor::GamepadMonitor(QObject *parent,interface_lcm_interface* control_lcm)
    : QObject(parent)
    , m_gamepad(0)
    , control_lcm(control_lcm)
{

    if(control_lcm == 0)
    {
        cout<<"[Gamepad] constructor got null control lcm\n";
        m_gamepad = nullptr;
        return;
    }

    auto gamepads = QGamepadManager::instance()->connectedGamepads();
    if (gamepads.isEmpty()) {
        cout<<"[Gamepad] no gamepad detected!\n";
        m_gamepad = nullptr;
        return;
    }

    m_gamepad = new QGamepad(*gamepads.begin(), this);
//    connect(m_gamepad, &QGamepad::axisLeftXChanged, this, [](double value){
//        cout << "Left X" << value <<"\n";
//    });
//    connect(m_gamepad, &QGamepad::axisLeftYChanged, this, [](double value){
//        cout << "Left Y" << value<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::axisRightXChanged, this, [&](double value){
//        //cout << "Right X" << value<<"\n";
//        control_lcm->main_control_settings.omega_des[2] = -value/2;
//    });
//    connect(m_gamepad, &QGamepad::axisRightYChanged, this, [&](double value){
//        //cout << "Right Y" << value<<"\n";
//        control_lcm->main_control_settings.v_des[0] = -value;
//    });
//    connect(m_gamepad, &QGamepad::buttonAChanged, this, [](bool pressed){
//        cout << "Button A" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonBChanged, this, [](bool pressed){
//        cout << "Button B" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonXChanged, this, [](bool pressed){
//        cout << "Button X" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonYChanged, this, [](bool pressed){
//        cout << "Button Y" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonL1Changed, this, [](bool pressed){
//        cout << "Button L1" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonR1Changed, this, [](bool pressed){
//        cout << "Button R1" << pressed<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonL2Changed, this, [](double value){
//        cout << "Button L2: " << value<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonR2Changed, this, [](double value){
//        cout << "Button R2: " << value<<"\n";
//    });
//    connect(m_gamepad, &QGamepad::buttonSelectChanged, this, [](bool pressed){
//        cout << "Button Select" << pressed<<"\n";
//    });
    connect(m_gamepad, &QGamepad::buttonBChanged, this, &GamepadMonitor::handle_estop);
    connect(m_gamepad, &QGamepad::buttonStartChanged, this, &GamepadMonitor::handle_start);

    //    connect(m_gamepad, &QGamepad::buttonStartChanged, this, [](bool pressed){
    //        //cout << "Button Start" << pressed<<"\n";
    //        handle_start(pressed);
    //    });

//    connect(m_gamepad, &QGamepad::buttonGuideChanged, this, [](bool pressed){
//        cout << "Button Guide" << pressed<<"\n";
//    });
}

double GamepadMonitor::get_axis(int axis)
{
    if(m_gamepad == nullptr)
        return 0.0;

    if(axis < 0 || axis > 3)
    {
        cout<<"[Gamepad Monitor] Error: get_axis called with out of bounds axis: "<<axis<<"\n";
        return 0.f;
    }

    if(axis == 0)
        return m_gamepad->axisLeftX();
    else if(axis == 1)
        return m_gamepad->axisLeftY();
    else if(axis == 2)
        return m_gamepad->axisRightX();
    else
        return m_gamepad->axisRightY();
}


QGamepad* GamepadMonitor::get_gamepad()
{
    return m_gamepad;
}

double GamepadMonitor::joystickmap(double in, double min, double max)
{
    return (in + 1.f) * (max - min)/(2.f) + min;
}

void GamepadMonitor::handle_start(bool pressed)
{
    if(estop)
    {
        control_lcm->main_control_settings.mode = 0;
        return;
    }
    if(!pressed) return;
    int control_mode = control_lcm->main_control_settings.mode;
    if(control_mode == 0) control_lcm->main_control_settings.mode = 1;
    if(control_mode == 1) control_lcm->main_control_settings.mode = 3;
    if(control_mode == 3) control_lcm->main_control_settings.mode = 6;
    if(control_mode == 6) control_lcm->main_control_settings.mode = 3;

}

void GamepadMonitor::handle_estop(bool pressed)
{
    if(!pressed) return;
    control_lcm->main_control_settings.mode = 0;
    estop = true;
}

GamepadMonitor::~GamepadMonitor()
{
    if(m_gamepad == nullptr)
        return;
    delete m_gamepad;
}
