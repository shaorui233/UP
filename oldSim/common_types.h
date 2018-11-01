#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

// contains standard types and magic numbers used in the GUI and simulator
#include <stdint.h>

// NOTE FRICTION IS CHAGNED!!!!!!!
// magic numbers go here
#define NUM_CONTACT_PTS 16
#define NUM_LEGS 4
#define GYRO_DIST .002
#define ACC_DIST .005


#define C3_PASSIVE_DAMPING 1.9
#define C3_DRY_FRICTION 1

#define MINI_PASSIVE_DAMPING 0.2
#define MINI_DRY_FRICTION 0.2

#define C3_L1 0.045
#define C3_L2 0.342
#define C3_L3 0.345

#define MINI_L1 .062
#define MINI_L2 .209
#define MINI_L3 .175

#define KT_MINI_CHEETAH (0.45 / 6)*(3/2)
#define KT_CHEETAH_3    0.4 * (3/2)

#define GR_CHEETAH_3_HIP_ABAD 7.66666
#define GR_MINI_CHEETAH_HIP_ABAD 6.0

#define GR_CHEETAH_3_KNEE 8.846
#define GR_MINI_CHEETAH_KNEE 9.33

#define R_CHEETAH_3 0.6
#define R_MINI_CHEETAH 0.173

// avoids "unused parameter" warning when used on unused parameters
#define UNUSED(expr) (void)(expr)

typedef double dbl;
typedef float flt;




// floating point type used for the simulator - currently using doubles
typedef double sim_flt;

//floating point type used when interfacing with MATLAB
typedef double mfp;
// integer type used when interfacing with MATLAB
typedef int mint;

//floating point type used in TI board emulation
typedef float ti_flt;

// usual stdint types
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;


#endif // COMMON_TYPES_H
