#ifndef ROBOT_LCM_HANDLER_H
#define ROBOT_LCM_HANDLER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include <cheetahlcm/monitor_info_t.hpp>
#include <cheetahlcm/cheetah_graphics_t.hpp>
#include <cheetahlcm/error_t.hpp>

class robot_lcm_handler
{
public:
    robot_lcm_handler();
    ~robot_lcm_handler();
    void handleMontitor(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const cheetahlcm::monitor_info_t* msg);

    void handleGraphics(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const cheetahlcm::cheetah_graphics_t* msg);

    void handleError(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const cheetahlcm::error_t* msg);

    cheetahlcm::error_t error;
    cheetahlcm::cheetah_graphics_t gfx;
    cheetahlcm::monitor_info_t monitor;

    int n_error = 0;
    int n_gfx = 0;
    int n_monitor = 0;

private:

    void run_lcm();
    bool want_shutdown = false;
    lcm::LCM lcm;
    std::thread* lcm_thread;

};

#endif // ROBOT_LCM_HANDLER_H
