#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p[i];
        this->v(i) = v[i];
        this->w(i) = w[i];
    }
    this->q.w() = q[0];
    this->q.x() = q[1];
    this->q.y() = q[2];
    this->q.z() = q[3];
    this->yaw = yaw;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw);
    fpt ys = sin(yaw);

    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    Matrix<fpt,3,1> Id;
    Id << 0.3f, 2.1f, 2.1f;
    I_body.diagonal() = Id;

    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



