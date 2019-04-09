#ifndef OPTIMIZATIONTHREAD_H
#define OPTIMIZATIONTHREAD_H

#include <unistd.h>
#include <qpOASES.hpp>
#include <iostream>

#include "MyThreadClass.hpp"

USING_NAMESPACE_QPOASES

class OptimizationThread : public MyThreadClass{
   public:
      OptimizationThread( QProblem QP_arg, 
                int_t* nWSR_arg, 
               real_t* H_arg, 
               real_t* g_arg, 
               real_t* A_arg, 
               real_t* lb_arg,
               real_t* ub_arg, 
               real_t* lbA_arg, 
               real_t* ubA_arg, 
               real_t* xOpt_arg, 
               real_t* yOpt_arg,
                 bool* qpFinished_arg);

      ~OptimizationThread() {}

      QProblem QProblemObject;
      int_t nWSR, nWSR_fixed;
      real_t *H, *A, *g, *lb, *ub, *lbA, *ubA, *xOpt, *yOpt;    
      bool *qpFinished;

      void solveQP_init();    
      void solveQP_hotstart();
      void printSolution();

      void InternalThreadEntry()
      {
         solveQP_init();
         *qpFinished = 1;
      };
};

#endif