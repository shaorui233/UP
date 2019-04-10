#include "OptimizationThread.hpp"

OptimizationThread::OptimizationThread( QProblem QP_arg, 
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
                 bool* qpFinished_arg)
{
  // Assign argument pointers to OptimizationThread member variables
  QProblemObject = QP_arg;
  Options options;  
  options.printLevel = PL_NONE;
  QProblemObject.setOptions( options ); 

  nWSR = *nWSR_arg;
  nWSR_fixed = nWSR;

  H = H_arg;
  A = A_arg;
  g = g_arg;
  lb = lb_arg;  
  ub = ub_arg;
  lbA = lbA_arg;
  ubA = ubA_arg;

  xOpt = xOpt_arg;
  yOpt = yOpt_arg;  
  qpFinished = qpFinished_arg;
}

void OptimizationThread::solveQP_init()
{ 
  QProblemObject.init(H,g,A,lb,ub,lbA,ubA,nWSR);     
  nWSR = nWSR_fixed;  
  QProblemObject.getPrimalSolution( xOpt);
  // std::cout << "xOpt[0] = " << xOpt[0];
  // std::cout << "lb[0] = " << lb[0];
  // printSolution();
  // std::cout << "\nQP solved = " << QProblemObject.isSolved() << "inFeasible = " << QProblemObject.isInfeasible();

  QProblemObject.reset();
  // usleep(30000000ul);
}

void OptimizationThread::solveQP_hotstart()
{ 
  QProblemObject.hotstart( g,lb,ub,lbA,ubA,nWSR);
  nWSR = nWSR_fixed; 
  QProblemObject.getPrimalSolution( xOpt );  
}

void OptimizationThread::printSolution()
{
  QProblemObject.getDualSolution( yOpt );

  printf("\nxOpt = [");
  for(int i = 0; i< QProblemObject.getNV(); i++) {printf("%e ",xOpt[i]); }
  printf("];\n");

  
  QProblemObject.printProperties();


  //printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
  //    xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],QProblemObject.getObjVal() );
}



