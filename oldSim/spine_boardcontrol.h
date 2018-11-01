#ifndef SPINE_BOARDCONTROL_H
#define SPINE_BOARDCONTROL_H

#include "common_types.h"
#include "cheetahlcm/spi_command_t.hpp"
#include "cheetahlcm/spi_data_t.hpp"

class spine_boardcontrol
{
public:
    spine_boardcontrol(ti_flt side_sign, s32 board);
    void run_spine_board_iteration();
    void reset_spine_board_data();
    void reset_spine_board_command();
    cheetahlcm::spi_command_t* cmd = nullptr;
    cheetahlcm::spi_data_t* data = nullptr;
    ti_flt torque_out[3];

private:
    ti_flt side_sign;
    s32 board_num;
    const ti_flt max_torque[3] =  {17.f, 17.f, 26.f}; // TODO CHECK WITH BEN
    const ti_flt wimp_torque[3] = {6.f,  6.f,  6.f }; // TODO CHECK WITH BEN
    const ti_flt disabled_torque[3] = {0.f, 0.f, 0.f};
    const float q_limit_p[3] = {1.5f, 5.0f, 0.f};
    const float q_limit_n[3] = {-1.5f, -5.0f, 0.f};
    const float kp_softstop = 100.f;
    const float kd_softstop = 0.4f;
    s32 iter_counter = 0;
};

#endif // SPINE_BOARDCONTROL_H
