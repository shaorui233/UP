#include "BalanceController.hpp"

#include <math.h>
#include <time.h>
#include <sys/time.h>

void getSysTime(timespec *ts)
{

#if defined(_POSIX_TIMERS) && !defined(__APPLE__)
    if (clock_gettime(CLOCK_REALTIME, ts) != 0)
    {
        throw ts;
    }
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ts->tv_sec = tv.tv_sec;
    ts->tv_nsec = 1000*tv.tv_usec;
#endif
}

double timeDiff(const timespec & t1, const timespec & t2) {
  return ((double) t2.tv_sec - t1.tv_sec) + (1.0e-9*((double) t2.tv_nsec - t1.tv_nsec));
}



int main () 
{
   BalanceController balanceControllerObject;

   double xfb[13] = {1,0,0,0,0,0,0.8,0,0,0,0,0,0};
   double p_feet_world[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
   double p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], four[4], twelve[12];

   timespec t1,t2;

   getSysTime(&t1);
   for (int i = 0; i < 1000; ++i)
   {
        balanceControllerObject.updateProblemData( xfb,
                                             p_feet_world,
                                             p_des, 
                                             p_act, 
                                             v_des, 
                                             v_act,
                                             O_err, four, twelve, twelve);
   }
   getSysTime(&t2);
   std::cout<< "Time " << timeDiff(t1,t2) *1000 << std::endl;


   double xOpt[12];
   getSysTime(&t1);
   for (int i = 0; i < 1000; ++i)
   {
      balanceControllerObject.solveQP(xOpt);
   }
   getSysTime(&t2);
   std::cout<< "Time " << timeDiff(t1,t2) *1000 << std::endl;

   for(int i = 0; i < 12; i ++)
   {
   	 std::cout << "xOpt[" << i << "] = " << xOpt[i] << "\n";
   }

   return 0;
}