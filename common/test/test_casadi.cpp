#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <casadi/casadi.hpp>

using namespace casadi;
using namespace std;

TEST(casadi, rocket_ipopt){
  cout << "program started" << endl;

  // Dimensions
  int nu = 20;  // Number of control segments
  int nj = 100; // Number of integration steps per control segment

  // optimization variable
  SX u = SX::sym("u", nu); // control

  SX s_0 = 0; // initial position
  SX v_0 = 0; // initial speed
  SX m_0 = 1; // initial mass

  SX dt = 10.0/(nj*nu); // time step
  SX alpha = 0.05; // friction
  SX beta = 0.1; // fuel consumption rate

  // Trajectory
  vector<SX> s_k, v_k, m_k;

  // Integrate over the interval with Euler forward
  SX s = s_0, v = v_0, m = m_0;
  for(int k=0; k<nu; ++k){
    for(int j=0; j<nj; ++j){
      s += dt*v;
      v += dt / m * (u(k)- alpha * v*v);
      m += -dt * beta*u(k)*u(k);
    }
    s_k.push_back(s);
    v_k.push_back(v);
    m_k.push_back(m);
  }
  SX s_all=vertcat(s_k), v_all=vertcat(v_k), m_all=vertcat(m_k);

  // Objective function
  SX f = dot(u, u);

  // Terminal constraints
  SX g = vertcat(s, v, v_all);

  // Create the NLP
  SXDict nlp = {{"x", u}, {"f", f}, {"g", g}};

  // Allocate an NLP solver and buffers
  Function solver = nlpsol("solver", "ipopt", nlp);

  // Bounds on g
  vector<double> gmin = {10, 0};
  vector<double> gmax = {10, 0};
  gmin.resize(2+nu, -numeric_limits<double>::infinity());
  gmax.resize(2+nu, 1.1);

  // Solve the problem
  DMDict arg = {{"lbx", -10},
    {"ubx", 10},
    {"x0", 0.4},
    {"lbg", gmin},
    {"ubg", gmax}};
  DMDict res = solver(arg);

  // Print the optimal cost
  double cost(res.at("f"));
  cout << "optimal cost: " << cost << endl;

  // Print the optimal solution
  vector<double> uopt(res.at("x"));
  cout << "optimal control: " << uopt << endl;

  // Get the state trajectory
  Function xfcn("xfcn", {u}, {s_all, v_all, m_all});
  vector<double> sopt, vopt, mopt;
  xfcn({uopt}, {&sopt, &vopt, &mopt});
  cout << "position: " << sopt << endl;
  cout << "velocity: " << vopt << endl;
  cout << "mass:     " << mopt << endl;

  bool b_save_matlab_file(false);
  if(b_save_matlab_file){
    // Create Matlab script to plot the solution
    ofstream file;
    string filename = "rocket_ipopt_results.m";
    file.open(filename.c_str());
    file << "% Results file from " __FILE__ << endl;
    file << "% Generated " __DATE__ " at " __TIME__ << endl;
    file << endl;
    file << "cost = " << cost << ";" << endl;
    file << "u = " << uopt << ";" << endl;

    // Save results to file
    file << "t = linspace(0,10.0," << nu << ");"<< endl;
    file << "s = " << sopt << ";" << endl;
    file << "v = " << vopt << ";" << endl;
    file << "m = " << mopt << ";" << endl;

    // Finalize the results file
    file << endl;
    file << "% Plot the results" << endl;
    file << "figure(1);" << endl;
    file << "clf;" << endl << endl;

    file << "subplot(2,2,1);" << endl;
    file << "plot(t,s);" << endl;
    file << "grid on;" << endl;
    file << "xlabel('time [s]');" << endl;
    file << "ylabel('position [m]');" << endl << endl;

    file << "subplot(2,2,2);" << endl;
    file << "plot(t,v);" << endl;
    file << "grid on;" << endl;
    file << "xlabel('time [s]');" << endl;
    file << "ylabel('velocity [m/s]');" << endl << endl;

    file << "subplot(2,2,3);" << endl;
    file << "plot(t,m);" << endl;
    file << "grid on;" << endl;
    file << "xlabel('time [s]');" << endl;
    file << "ylabel('mass [kg]');" << endl << endl;

    file << "subplot(2,2,4);" << endl;
    file << "plot(t,u);" << endl;
    file << "grid on;" << endl;
    file << "xlabel('time [s]');" << endl;
    file << "ylabel('Thrust [kg m/s^2]');" << endl << endl;

    file.close();
    cout << "Results saved to \"" << filename << "\"" << endl;
  }
}

TEST(casadi, part1){
  /** Test problem
   *
   *    min x0^2 + x1^2
   *    s.t.    x0 + x1 - 10 = 0
   */

  // Optimization variables
  MX x = MX::sym("x", 2);

  // Objective
  MX f = x(0)*x(0) + x(1)*x(1);

  // Constraints
  MX g = x(0)+x(1)-10;

  // Create an NLP solver instance
  Function solver = nlpsol("solver", "ipopt", {{"x", x}, {"f", f}, {"g", g}});

  // Generate C code for the NLP functions
  solver.generate_dependencies("nlp.c");

  // Just-in-time compilation?
  bool jit = false;
  if (jit) {
    // Create a new NLP solver instance using just-in-time compilation
    solver = nlpsol("solver", "ipopt", "nlp.c");
  } else {
    // Compile the c-code
    int flag = system("gcc -fPIC -shared -O3 nlp.c -o nlp.so");
    casadi_assert(flag==0, "Compilation failed");

    // Create a new NLP solver instance from the compiled code
    solver = nlpsol("solver", "ipopt", "nlp.so");
  }

  // Bounds and initial guess
  std::map<std::string, DM> arg, res;
  arg["lbx"] = -DM::inf();
  arg["ubx"] =  DM::inf();
  arg["lbg"] =  0;
  arg["ubg"] =  DM::inf();
  arg["x0"] = 0;

  // Solve the NLP
  res = solver(arg);

  // Print solution
  cout << "-----" << endl;
  cout << "objective at solution = " << res.at("f") << endl;
  cout << "primal solution = " << res.at("x") << endl;
  cout << "dual solution (x) = " << res.at("lam_x") << endl;
  cout << "dual solution (g) = " << res.at("lam_g") << endl;
}

