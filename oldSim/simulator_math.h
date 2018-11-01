#ifndef SIMULATOR_MATH_H
#define SIMULATOR_MATH_H

#include "common_types.h"
#include <cmath>


// simulator math functions
// replace macros from old simulator with template functions

class SimulatorMath
{
public:

    template <typename T1, typename T2, typename T3>
    static void apply_cartesian_tensor(T1 A[3][3], T2* b, T3* Ab)
    {
        Ab[0] = A[0][0]*b[0]+A[0][1]*b[1]+A[0][2]*b[2];
        Ab[1] = A[1][0]*b[0]+A[1][1]*b[1]+A[1][2]*b[2];
        Ab[2] = A[2][0]*b[0]+A[2][1]*b[1]+A[2][2]*b[2];
    }

    template <typename T1, typename T2>
    static void add_cartesian_vector(T1* a, T2* b)
    {
        a[0]+=b[0];
        a[1]+=b[1];
        a[2]+=b[2];
    }

    template <typename T1, typename T2, typename T3>
    static void cross_product(T1* a, T2* b, T3* c)
    {
        c[0] = a[1]*b[2]-a[2]*b[1];
        c[1] = -a[0]*b[2]+a[2]*b[0];
        c[2] = a[0]*b[1]-a[1]*b[0];
    }


    static void cartesian_diff(sim_flt* a, sim_flt* b, sim_flt* c);
    static sim_flt cartesian_norm(sim_flt* a);
    static sim_flt cartesian_dot(sim_flt* a, sim_flt* b);
    static void QuatToR(const sim_flt quat[4], sim_flt R[3][3] );
    static void QuatToRpy(const sim_flt quat[4], sim_flt rpy[]);
    static void quatProduct(sim_flt a[4], sim_flt b[4], float *c);
    static void OmegabToRpyRates( const sim_flt omegab[], const sim_flt rpy[3], sim_flt rpyRates[] );
};

#endif // SIMULATOR_MATH_H
