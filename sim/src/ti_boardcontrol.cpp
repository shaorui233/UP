#include "ti_boardcontrol.h"

#include <iostream>
#include <cmath>
#include <algorithm>



TI_BoardControl::TI_BoardControl(ti_flt side_sign)
{
    std::cout<<"[Cheetah Control] Hello! I am the TI board with side sign: "<<side_sign<<"\n";
    this->side_sign = side_sign;
    data = &data_structure;
}

void TI_BoardControl::set_link_lengths(ti_flt l1, ti_flt l2, ti_flt l3)
{
    link_lengths_set = true;
    this->l1 = l1;
    this->l2 = l2;
    this->l3 = l3;
}

void TI_BoardControl::reset_ti_board_data()
{
    std::cout<<"[TI Board] Resetting TI board data.\n";
    std::fill_n(data_structure.tau_des,3,0);
    data_structure.loop_count_ti = 0;
    data_structure.ethercat_count_ti = 0;
    data_structure.microtime_ti = 0;
}

void TI_BoardControl::reset_ti_board_command()
{
    std::cout<<"[TI Board] Resetting TI board command.\n";
    command.position_des[0] = .1;
    command.position_des[1] = side_sign*.1;
    command.position_des[2] = -.5;

    sim_flt mult = 5;
    std::fill_n(command.velocity_des , 3  , 0 );
    std::fill_n(command.kp   , 3, 3*mult );
    std::fill_n(command.kd   , 3, .1*mult );
    std::fill_n(data_structure.tau_des , 3, 0      );
}

void TI_BoardControl::run_ti_board_iteration()
{
    if(!link_lengths_set)
    {
        std::cout<<"[TI Board] Error.  Link lengths haven't been set.\n";
        return;
    }

    if(command.enable == 1 && command.max_torque > 1e-9)
    {
       impedanceControl(side_sign,
                data->q, data->dq, command.position_des, command.velocity_des,
                command.kp   , command.kd,
                command.force_ff , command.tau_ff,
                data->position, data->velocity, data->force, data->tau_des);

        for (int i = 0; i < 3; ++i) {
            if(abs(data->tau_des[i]) > command.max_torque) {
                data->tau_des[i] = data->tau_des[i]/abs(data->tau_des[i])*command.max_torque;
            }
        }
    }
    else {
        ti_flt Jacobian[3][3];
        kinematics(side_sign, data->q, data->dq, data->position, data->velocity, Jacobian);
        data->force[0]   = 0;   data->force[1]   = 0;   data->force[2] = 0;
        data->tau_des[0] = 0;   data->tau_des[1] = 0;   data->tau_des[2] = 0;
    }
    data->loop_count_ti += 1;
}


void TI_BoardControl::impedanceControl(const ti_flt side_sign, const ti_flt q[3], const ti_flt dq[3],
                      const ti_flt position_des[3], const ti_flt velocity_des[3],
                      const ti_flt kp[3] 		  , const ti_flt kd[3],
                      const ti_flt force_bias[3]  , const ti_flt torque_bias[3],
                      ti_flt * position, ti_flt * velocity, ti_flt * force, ti_flt * torque )
{
    ti_flt Jacobian[3][3];
    kinematics(side_sign, q, dq, position, velocity, Jacobian);

    force[0] = kp[0]*(position_des[0]-position[0]) + kd[0]*(velocity_des[0] - velocity[0]) + force_bias[0];
    force[1] = kp[1]*(position_des[1]-position[1]) + kd[1]*(velocity_des[1] - velocity[1]) + force_bias[1];
    force[2] = kp[2]*(position_des[2]-position[2]) + kd[2]*(velocity_des[2] - velocity[2]) + force_bias[2];

    torque[0] = (force[0]*Jacobian[0][0] + force[1]*Jacobian[1][0] + force[2]*Jacobian[2][0]) + torque_bias[0];
    torque[1] = (force[0]*Jacobian[0][1] + force[1]*Jacobian[1][1] + force[2]*Jacobian[2][1]) + torque_bias[1];
    torque[2] = (force[0]*Jacobian[0][2] + force[1]*Jacobian[1][2] + force[2]*Jacobian[2][2]) + torque_bias[2];
}

void TI_BoardControl::kinematics(const ti_flt side_sign, const ti_flt q[3], const ti_flt dq[3], ti_flt * p, ti_flt * v, ti_flt J[][3])
{

//    ti_flt l1 = 0.045;
//    ti_flt l2 = .342;
//    ti_flt l3 = .345;

//    if(get_cheetah() == 4)
//    {
//        l1 = .062;
//        l2 = .209;
//        l3 = .175;
//    }

    const ti_flt s1 = sin(q[0]), s2 = sin(q[1]), s3 = sin(q[2]);
    const ti_flt c1 = cos(q[0]), c2 = cos(q[1]), c3 = cos(q[2]);
    const ti_flt c23 = c2*c3 - s2*s3;
    const ti_flt s23 = s2*c3 + c2*s3;

    p[0] = l3*s23 + l2*s2;
    p[1] = l1*side_sign*c1 + l3*(s1*c23) + l2*c2*s1;
    p[2] = l1*side_sign*s1 - l3*(c1*c23) - l2*c1*c2;

    J[0][0] = 0;
    J[0][1] = l3*c23+ l2*c2;
    J[0][2] = l3*c23;
    J[1][0] =  l3*c1*c23 + l2*c1*c2 - l1*side_sign*s1 ;
    J[1][1] = -l3*s1*s23 - l2*s1*s2;
    J[1][2] = -l3*s1*s23;
    J[2][0] = l3*s1*c23 + l2*c2*s1 + l1*side_sign*c1;
    J[2][1] = l3*c1*s23 + l2*c1*s2;
    J[2][2] = l3*c1*s23;


    v[0] = 0; v[1] = 0; v[2] = 0;
    int i, j;
    for(i = 0; i<3 ; i++)
    {
        for(j = 0; j<3; j++)
        {
            v[i] += J[i][j]*dq[j];
        }
    }
}
