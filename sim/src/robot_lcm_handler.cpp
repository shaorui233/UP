#include "robot_lcm_handler.h"
#include <stdio.h>

robot_lcm_handler::robot_lcm_handler():
lcm("udpm://239.255.76.67:7667?ttl=1")
{
    if(lcm.good())
        printf("[Graphics LCM] Graphics LCM is good!\n");
    else
        printf("[ERROR: Graphics LCM] LCM isn't working.\n");

    lcm.subscribe("CHEETAH_cheetah_graphics", &robot_lcm_handler::handleGraphics, this);
    lcm.subscribe("CHEETAH_error", &robot_lcm_handler::handleError, this);
    lcm.subscribe("CHEETAH_monitor_info", &robot_lcm_handler::handleMontitor, this);

    memset(&error,0,sizeof(error));
    memset(&gfx,0,sizeof(gfx));
    memset(&monitor,0,sizeof(monitor));

    gfx.q[0] = 1.2; // just to make it do something.

    lcm_thread = new std::thread(&robot_lcm_handler::run_lcm, this);
}


robot_lcm_handler::~robot_lcm_handler()
{
    want_shutdown = true;
    lcm_thread->join();
    delete lcm_thread;
}


void robot_lcm_handler::run_lcm()
{
    printf("[Graphics LCM Handler Thread] Hello!\n");
    for(;;)
    {
        if(want_shutdown)
            return;
        lcm.handleTimeout(10);
    }
}

void robot_lcm_handler::handleError(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const cheetahlcm::error_t *msg)
{
    error = *msg;
    n_error++;
}

void robot_lcm_handler::handleGraphics(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const cheetahlcm::cheetah_graphics_t *msg)
{
    gfx = *msg;
    n_gfx++;
}

void robot_lcm_handler::handleMontitor(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const cheetahlcm::monitor_info_t *msg)
{
    monitor = *msg;
    n_monitor++;
}


