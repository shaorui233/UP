#include "simulator_math.h"

// from old simulator macros

//void SimulatorMath::apply_cartesian_tensor(sim_flt **A, sim_flt *b, sim_flt *Ab)
//{
//    Ab[0] = A[0][0]*b[0]+A[0][1]*b[1]+A[0][2]*b[2];
//    Ab[1] = A[1][0]*b[0]+A[1][1]*b[1]+A[1][2]*b[2];
//    Ab[2] = A[2][0]*b[0]+A[2][1]*b[1]+A[2][2]*b[2];
//}

//void SimulatorMath::add_cartesian_vector(sim_flt *a, sim_flt *b)
//{
//    a[0]+=b[0];
//    a[1]+=b[1];
//    a[2]+=b[2];
//}

//void SimulatorMath::cross_product(sim_flt *a, sim_flt *b, sim_flt *c)
//{
//    c[0] = a[1]*b[2]-a[2]*b[1];
//    c[1] = -a[0]*b[2]+a[2]*b[0];
//    c[2] = a[0]*b[1]-a[1]*b[0];
//}

void SimulatorMath::cartesian_diff(sim_flt *a, sim_flt *b, sim_flt *c)
{
    c[0] = b[0] - a[0];
    c[1] = b[1] - a[1];
    c[2] = b[2] - a[2];
}

sim_flt SimulatorMath::cartesian_norm(sim_flt *a)
{
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

sim_flt SimulatorMath::cartesian_dot(sim_flt *a, sim_flt *b)
{
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

void SimulatorMath::QuatToR(const sim_flt quat[], sim_flt R[3][3])
{
    sim_flt e0 = quat[0];
    sim_flt e1 = quat[1];
    sim_flt e2 = quat[2];
    sim_flt e3 = quat[3];

    R[0][0] = 1-2*(e2*e2+e3*e3) ; 	R[0][1] = 2*(e1*e2-e0*e3);		R[0][2] = 2*(e1*e3 + e0*e2) ;
    R[1][0] = 2*(e1*e2+e0*e3) ;  	R[1][1] = 1-2*(e1*e1+e3*e3);	R[1][2] = 2*(e2*e3 - e0*e1) ;
    R[2][0] = 2*(e1*e3 -e0*e2);  	R[2][1] = 2*(e2*e3+e0*e1);     	R[2][2] = 1-2*(e1*e1+e2*e2) ;
}

void SimulatorMath::QuatToRpy(const sim_flt quat[4], sim_flt rpy[]) {
    sim_flt q0 = quat[0];
    sim_flt q1 = quat[1];
    sim_flt q2 = quat[2];
    sim_flt q3 = quat[3];

    rpy[0] = atan2(2*q2*q3+2*q0*q1,q3*q3-q2*q2-q1*q1+q0*q0);
    rpy[1] = -asin(2*q1*q3-2*q0*q2);
    rpy[2] = atan2(2*q1*q2+2*q0*q3,q1*q1+q0*q0-q3*q3-q2*q2);
}

void SimulatorMath::OmegabToRpyRates( const sim_flt omegab[], const sim_flt rpy[3], sim_flt rpyRates[] ) {
    sim_flt wx = omegab[0], wy = omegab[1], wz = omegab[2];
    sim_flt  r =    rpy[0], p  =    rpy[1];//, y  =    rpy[2];

    rpyRates[0] = cos(p)*wx + sin(p)*sin(r)*wy + cos(r)*sin(p)*wz;
    rpyRates[1] = 	   0*wx + cos(p)*cos(r)*wy - cos(p)*sin(r)*wz;
    rpyRates[2] = 	   0*wx + sin(r)       *wy + cos(r)       *wz;

    rpyRates[0]/=cos(p);
    rpyRates[1]/=cos(p);
    rpyRates[2]/=cos(p);
}

void SimulatorMath::quatProduct(sim_flt a[], sim_flt b[], float *c)
{
     c[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
     c[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
     c[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
     c[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
