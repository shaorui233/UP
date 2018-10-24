#ifndef TI_BOARDCONTROL_H
#define TI_BOARDCONTROL_H

#include "common_types.h"

// class for a simulated TI board
// replaces the old TI board control code, which required keeping track of the command/data state data.
struct TiBoardCommand {
    ti_flt position_des[3];
    ti_flt velocity_des[3];
    ti_flt kp[3];
    ti_flt kd[3];
    ti_flt force_ff[3];
    ti_flt tau_ff[3];
    s32 enable;
    ti_flt max_torque;
} ;

 struct TiBoardData {
    ti_flt position[3];
    ti_flt velocity[3];
    ti_flt force[3];
    ti_flt q[3];
    ti_flt dq[3];
    ti_flt tau[3];
    ti_flt tau_des[3];
    u32 loop_count_ti;
    u32 ethercat_count_ti;
    u32 microtime_ti;
} ;

class TI_BoardControl
{
public:
    TI_BoardControl(ti_flt side_sign);
    void run_ti_board_iteration();
    void reset_ti_board_data();
    void reset_ti_board_command();
    void set_link_lengths(ti_flt l1, ti_flt l2, ti_flt l3);
    TiBoardCommand command;
    TiBoardData data_structure;

    // for Ben's TI board code that uses pointers
    TiBoardData* data;


private:
    void kinematics(const ti_flt side_sign,
                    const ti_flt q[3], const ti_flt dq[3], ti_flt * p, ti_flt * v, ti_flt J[][3]);
    void impedanceControl(const ti_flt side_sign      ,
                          const ti_flt q[3], const ti_flt dq[3],
                          const ti_flt position_des[3], const ti_flt velocity_des[3],
                          const ti_flt kp[3] 		  , const ti_flt kd[3],
                          const ti_flt force_bias[3]  , const ti_flt torque_bias[3],
                          ti_flt * position, ti_flt * velocity, ti_flt * force, ti_flt * torque );

    ti_flt side_sign;
    ti_flt l1, l2, l3;
    bool link_lengths_set = false;
};

#endif // TI_BOARDCONTROL_H
