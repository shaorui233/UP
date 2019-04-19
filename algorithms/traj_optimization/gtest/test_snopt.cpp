#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

//#include "snoptProblem.hpp"
#include <snoptProblem.hpp>

using namespace std;

void usrFG(int    *Status, int *n,    double x[],
	   int    *needF,  int *lenF,  double F[],
	   int    *needG,  int *lenG,  double G[],
	   char      *cu,  int *lencu,
	   int    iu[],    int *leniu,
	   double ru[],    int *lenru) {
  int neG = 0;
  int jx1, jx2, ju, ode1, ode2, ObjRow;
  double alpha = 0.0, tf = 1.0;
  double  Fobj = 0.0, Gobj0 = 0.0;
  double Gobj1;

  int    nH    = *n / 3 - 1;
  double h     = tf / nH;

  jx1    = 0;
  jx2    = jx1 + nH + 1;
  ju     = jx2 + nH + 1;

  ode1   = 0;
  ode2   = ode1 + nH;
  ObjRow = ode2 + nH;

  for (int i = 0; i < nH; i++) {
    if (*needF > 0) {
      Fobj += alpha*h*pow(x[ju+i+1] - x[ju+i], 2);

      // subject to ode1 {i in 0..(nh-1]}
      F[ode1+i] = x[jx1+i+1] - x[jx1+i]
	- 0.5*h*( x[ju+i]  *(10.0*x[jx2+i]   - x[jx1+i])
		    + x[ju+i+1]*(10.0*x[jx2+i+1] - x[jx1+i+1]));
      // subject to ode2 {i in 0..[nh-1]}
      F[ode2+i] = x[jx2+i+1] - x[jx2+i]
	- 0.5*h*(x[ju+i]  *(x[jx1+i]   - 10.0*x[jx2+i])
		   - (1.0-x[ju+i])  *x[jx2+i]
		   +x[ju+i+1]*(x[jx1+i+1] - 10.0*x[jx2+i+1])
		   - (1.0-x[ju+i+1])*x[jx2+i+1]);
    }

    if (*needG > 0) {
      // Objective gradient.
      Gobj1  =  2.0*alpha*h*(x[ju+i+1] - x[ju+i]);
      G[neG] =  Gobj0 - Gobj1;
      neG    = neG + 1;
      Gobj0  =  Gobj1;

      // First ode constraint.
      G[neG] =  - 1.0 + 0.5*h*x[ju+i];
      neG    = neG + 1;

      G[neG] =    1.0 + 0.5*h*x[ju+i+1];
      neG    = neG + 1;

      G[neG] =        - 5.0*h*x[ju+i];
      neG    = neG + 1;

      G[neG] =        - 5.0*h*x[ju+i+1];
      neG    = neG + 1;

      G[neG] =        - 0.5*h*(10.0*x[jx2+i]   - x[jx1+i]);
      neG    = neG + 1;

      G[neG] =        - 0.5*h*(10.0*x[jx2+i+1] - x[jx1+i+1]);
      neG    = neG + 1;

      // second ode constraint.
      G[neG] =        - 0.5*h*x[ju+i];
      neG    = neG + 1;

      G[neG] =        - 0.5*h*x[ju+i+1];
      neG = neG + 1;

      G[neG] = - 1.0  + 0.5*h*(9.0*x[ju+i]   + 1.0);
      neG    = neG + 1;

      G[neG] =   1.0  + 0.5*h*(9.0*x[ju+i+1] + 1.0);
      neG    = neG + 1;

      G[neG] = - 0.5*h*(x[jx1+i]   - 9.0*x[jx2+i]);
      neG    = neG + 1;

      G[neG] = - 0.5*h*(x[jx1+i+1] - 9.0*x[jx2+i+1]);
      neG    = neG + 1;
    }
  }

  if (*needG > 0) {
    G[neG] = -Gobj0;
    neG    = neG + 1;
  }

  if (*needF > 0) {
    F[ObjRow] = Fobj;
  }
}


TEST(snopt, snopt_test){
  snoptProblemA catmixa;

  int Cold  = 0;

  int nH    = 1000;
  int n     = 3*(nH+1);
  int nCon  = 2*nH;
  int nF    = nCon + 1;

  int    ObjRow;
  double ObjAdd = -1.0;

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[nF];
  double *Flow   = new double[nF];
  double *Fupp   = new double[nF];
  double *Fmul   = new double[nF];
  int    *Fstate = new int[nF];

  int lenA   = 2;
  int *iAfun = new int[lenA];
  int *jAvar = new int[lenA];
  double *A  = new double[lenA];

  int lenG   = 14*nH + 1;
  int *iGfun = new int[lenG];
  int *jGvar = new int[lenG];

  int nS = 0, nInf = 0, neA = 0, neG = 0;
  int jx1, jx2, ju, ode1, ode2, Obj;
  double sInf;

  double inf = 1.0e20;

  jx1 = 0;
  jx2 = jx1 + nH + 1;
  ju  = jx2 + nH + 1;

  ode1 = 0;
  ode2 = ode1 + nH;
  Obj  = ode2 + nH;

  ObjRow = Obj;

  // Linear terms first
  iAfun[neA] = ObjRow;
  jAvar[neA] = jx1 + nH;
  A[neA]     = 1.0;
  neA        = neA + 1;

  iAfun[neA] = ObjRow;
  jAvar[neA] = jx2 + nH;
  A[neA]     = 1.0;
  neA        = neA + 1;


  for (int i = 0; i < nH; i++) {
    iGfun[neG] = ObjRow;
    jGvar[neG] = ju + i;
    neG        = neG + 1;

    // First ode constraint
    iGfun[neG] = ode1 + i;
    jGvar[neG] = jx1  + i;
    neG        = neG + 1;

    iGfun[neG] = ode1 + i;
    jGvar[neG] = jx1  + i + 1;
    neG        = neG + 1;

    iGfun[neG] = ode1 + i;
    jGvar[neG] = jx2  + i;
    neG        = neG + 1;

    iGfun[neG] = ode1 + i;
    jGvar[neG] = jx2  + i + 1;
    neG        = neG + 1;

    iGfun[neG] = ode1 + i;
    jGvar[neG] = ju   + i;
    neG        = neG + 1;

    iGfun[neG] = ode1 + i;
    jGvar[neG] = ju   + i + 1;
    neG        = neG + 1;

    // Second ode constraint
    iGfun[neG] = ode2 + i;
    jGvar[neG] = jx1  + i;
    neG        = neG + 1;

    iGfun[neG] = ode2 + i;
    jGvar[neG] = jx1  + i + 1;
    neG        = neG + 1;

    iGfun[neG] = ode2 + i;
    jGvar[neG] = jx2  + i;
    neG        = neG + 1;

    iGfun[neG] = ode2 + i;
    jGvar[neG] = jx2  + i + 1;
    neG        = neG + 1;

    iGfun[neG] = ode2 + i;
    jGvar[neG] = ju   + i;
    neG        = neG + 1;

    iGfun[neG] = ode2 + i;
    jGvar[neG] = ju   + i + 1;
    neG        = neG + 1;
  }

  iGfun[neG] = ObjRow;
  jGvar[neG] = ju + nH;
  neG        = neG + 1;

  // Set bounds, states and initial values.
  for (int i = 0; i <= nH; i++) {
    // x1 variable
    xlow[jx1+i]   = -inf;
    xupp[jx1+i]   =  inf;
    x[jx1+i]      =  1.0;
    xstate[jx1+i] =  0;

    // x2 variable
    xlow[jx2+i]   = -inf;
    xupp[jx2+i]   =  inf;
    x[jx2+i]      =  0.0;
    xstate[jx2+i] =  0;

    // u variable
    xlow[ju+i]   = 0.0;
    xupp[ju+i]   = 1.0;
    x[ju+i]      = 0.0;
    xstate[ju+i] = 0;
  }

  xlow[jx1] = 1.0;
  xupp[jx1] = 1.0;
  x[jx1]    = 1.0;

  xlow[jx2] = 0.0;
  xupp[jx2] = 0.0;
  x[jx2]    = 0.0;

  // Bounds on F (all equalities)
  for (int i = 0; i < nCon; i++) {
    Flow[i] = 0.0;
    Fupp[i] = 0.0;
    Fmul[i] = 0.0;
  }

  // Objective row
  Fmul[ObjRow] = 0.0;
  Flow[ObjRow] = -inf;
  Fupp[ObjRow] =  inf;

  catmixa.initialize     ("", 1);  // no print file, summary on
  catmixa.setProbName    ("catmix");

  catmixa.setPrintFile   ("catmix.out");  // ok now add a print file
  catmixa.setIntParameter("Verify level ", 3);

  catmixa.solve          (Cold, nF, n, ObjAdd, ObjRow, usrFG,
			  iAfun, jAvar, A, neA,
			  iGfun, jGvar, neG,
			  xlow, xupp, Flow, Fupp,
			  x, xstate, xmul,
			  F, Fstate, Fmul,
			  nS, nInf, sInf);

  delete []iAfun;  delete []jAvar;  delete []A;
  delete []iGfun;  delete []jGvar;

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;

}

void toyusrf(int    *Status, int *n,    double x[],
	     int    *needF,  int *neF,  double F[],
	     int    *needG,  int *neG,  double G[],
	     char      *cu,  int *lencu,
	     int    iu[],    int *leniu,
	     double ru[],    int *lenru) {
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // neF = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  //==================================================================
  F[0] =  x[1];
  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

void toyusrfg(int    *Status, int *n,    double x[],
	      int    *needF,  int *neF,  double F[],
	      int    *needG,  int *neG,  double G[],
	      char      *cu,  int *lencu,
	      int    iu[],    int *leniu,
	      double ru[],    int *lenru) {
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // neF = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //
  // The triples (g(k),iGfun(k),jGvar(k)), k = 1:neG, define
  // the sparsity pattern and values of the nonlinear elements
  // of the Jacobian.
  //==================================================================

  if (*needF > 0) {
    F[0] =  x[1]; //  Objective row
    F[1] =  x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }

  if (*needG > 0) {
    // iGfun[0] = 1
    // jGvar[0] = 0
    G[0] = 2*x[0];

    // iGfun[1] = 1
    // jGvar[1] = 1
    G[1] = 8*x[1];

    // iGfun[2] = 2
    // jGvar[2] = 0
    G[2] = 2*(x[0] - 2);

    // iGfun[3] = 2
    // jGvar[3] = 1
    G[3] = 2*x[1];
  }
}

TEST(snopt, snopt_toyprob){
  snoptProblemA ToyProb;

  // Allocate and initialize;
  int n     =  2;
  int neF   =  3;

  int nS = 0, nInf;
  double sInf;

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[neF];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];
  double *Fmul   = new double[neF];
  int    *Fstate = new int[neF];

  int    ObjRow  = 0;
  double ObjAdd  = 0;

  int Cold = 0, Basis = 1, Warm = 2;


  // Set the upper and lower bounds.
  xlow[0]   =  0.0;  xlow[1]   = -1e20;
  xupp[0]   = 1e20;  xupp[1]   =  1e20;
  xstate[0] =    0;  xstate[1] =  0;

  Flow[0] = -1e20; Flow[1] = -1e20; Flow[2] = -1e20;
  Fupp[0] =  1e20; Fupp[1] =   4.0; Fupp[2] =  5.0;
  Fmul[0] =   0;   Fmul[0] =   0;   Fmul[0] =    0;
  x[0]    = 1.0;
  x[1]    = 1.0;

  // Load the data for ToyProb ...
  ToyProb.initialize    ("", 1);      // no print file; summary on
  ToyProb.setPrintFile  ("Toy0.out"); // oh wait, i want a print file
  ToyProb.setProbName   ("Toy0");

  // snopta will compute the Jacobian by finite-differences.
  // snJac will be called  to define the
  // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
  ToyProb.setIntParameter("Derivative option", 0);
  ToyProb.setIntParameter("Verify level ", 3);

  // Solve the problem.
  // snJac is called implicitly in this case to compute the Jacobian.
  ToyProb.solve(Cold, neF, n, ObjAdd, ObjRow, toyusrf,
		xlow, xupp, Flow, Fupp,
		x, xstate, xmul, F, Fstate, Fmul,
		nS, nInf, sInf);


  // Reset the variables and solve ...
  printf("\nSolving toy1 problem using derivatives...\n");

  int lenA   = 6;
  int *iAfun = new int[lenA];
  int *jAvar = new int[lenA];
  double *A  = new double[lenA];

  int lenG   = 6;
  int *iGfun = new int[lenG];
  int *jGvar = new int[lenG];

  int neA, neG; // neA and neG must be defined when providing dervatives

  xstate[0] =   0;  xstate[1] = 0;
  Fmul[0]   =   0;  Fmul[0]   = 0; Fmul[0] =    0;
  x[0]      = 1.0;
  x[1]      = 1.0;


  // Provide the elements of the Jacobian explicitly.
  iGfun[0] = 1;
  jGvar[0] = 0;

  iGfun[1] = 1;
  jGvar[1] = 1;

  iGfun[2] = 2;
  jGvar[2] = 0;

  iGfun[3] = 2;
  jGvar[3] = 1;
  neG      = 4;

  iAfun[0] = 0;
  jAvar[0] = 1;
  A[0]     = 1.0;
  neA      = 1;

  ToyProb.setProbName    ("Toy1");         // Give the problem a new name for Snopt.

  ToyProb.setPrintFile   ("Toy1.out");
  ToyProb.setSpecsFile   ("sntoya.spc");
  ToyProb.setIntParameter("Derivative option", 1);
  ToyProb.setIntParameter("Major Iteration limit", 250);
  ToyProb.setIntParameter("Verify level ", 3);

  ToyProb.solve(Cold, neF, n, ObjAdd, ObjRow, toyusrfg,
		iAfun, jAvar, A, neA,
		iGfun, jGvar, neG,
		xlow, xupp, Flow, Fupp,
		x, xstate, xmul,
		F, Fstate, Fmul,
		nS, nInf, sInf);


  //for (int i = 0; i < n; i++){
    //cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  //}
  //for (int i = 0; i < neF; i++){
    //cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  //}

  ToyProb.solve(Warm, neF, n, ObjAdd, ObjRow, toyusrfg,
		iAfun, jAvar, A, neA,
		iGfun, jGvar, neG,
		xlow, xupp, Flow, Fupp,
		x, xstate, xmul,
		F, Fstate, Fmul,
		nS, nInf, sInf);

  //for (int i = 0; i < n; i++){
    //cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  //}
  //for (int i = 0; i < neF; i++){
    //cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  //}


  delete []iAfun;  delete []jAvar;  delete []A;
  delete []iGfun;  delete []jGvar;

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;
}
