#ifndef GAMEPADMONITOR_H
#define GAMEPADMONITOR_H

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include "interface_lcm_interface.h"

class QGamepad;

class GamepadMonitor : public QObject
{
    Q_OBJECT
public:
    explicit GamepadMonitor(QObject *parent = 0,interface_lcm_interface* control_lcm = 0);
    double get_axis(int axis);
    double joystickmap(double in, double min, double max);
    QGamepad* get_gamepad();
    ~GamepadMonitor();

private:


    QGamepad *m_gamepad;
    interface_lcm_interface* control_lcm;
    bool estop = false;


public slots:
    void handle_start(bool pressed);
    void handle_estop(bool pressed);

};

#endif // GAMEPADMONITOR_H
