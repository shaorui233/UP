#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <casadi/casadi.hpp>
using namespace casadi;

TEST(casadi, casadi_test){
    MX x = MX::sym("x",2); // Two states

    // Expression for ODE right-hand side
    MX z = 1-pow(x(1),2);
    MX rhs = vertcat(z*x(0)-x(1),x(0));

    MXDict ode;         // ODE declaration
    ode["x"]   = x;     // states
    ode["ode"] = rhs;   // right-hand side

    // // Construct a Function that integrates over 4s
    Function F = integrator("F","cvodes",ode,{{"tf",4}});
    
    // Start from x=[0;1]
    DMDict res = F(DMDict{{"x0",std::vector<double>{0,1}}});
}


TEST(casadi, casadi_dynamic_system){
    MX x = MX::sym("x",2); // Two states

    // Expression for ODE right-hand side
    MX z = 1-pow(x(1),2);
    MX rhs = vertcat(z*x(0)-x(1),x(0));

    MXDict ode;         // ODE declaration
    ode["x"]   = x;     // states
    ode["ode"] = rhs;   // right-hand side

    // Construct a Function that integrates over 4s
    Function F = integrator("F","cvodes",ode,{{"tf",4}});
    // Start from x=[0;1]
    DMDict res = F(DMDict{{"x0",std::vector<double>{0,1}}});
}

TEST(casadi, ipopt){
    MX x = MX::sym("x",2); // Two states
    MX p = MX::sym("p");   // Free parameter

    // Expression for ODE right-hand side
    MX z = 1-pow(x(1),2);
    MX rhs = vertcat(z*x(0)-x(1)+2*tanh(p),x(0));
    // ODE declaration with free parameter
    MXDict ode = {{"x",x},{"p",p},{"ode",rhs}};

    // Construct a Function that integrates over 1s
    Function F = integrator("F","cvodes",ode,{{"tf",1}});

    // Control vector
    MX u = MX::sym("u",4,1);

    x = DM(std::vector<double>{0,1});  // Initial state
    for (int k=0;k<4;++k) {
        // Integrate 1s forward in time:
        // call integrator symbolically
        MXDict res = F({{"x0",x},{"p",u(k)}});
        x = res["xf"];
    }

    // NLP declaration
    MXDict nlp = {{"x",u},{"f",dot(u,u)},{"g",x}};

    // Solve using IPOPT
    Function solver = nlpsol("solver","ipopt",nlp);
    DMDict res = solver(DMDict{{"x0",0.2},{"lbg",0},{"ubg",0}});
}


/*
TEST(casadi, snopt){
    MX x = MX::sym("x",2); // Two states
    MX p = MX::sym("p");   // Free parameter

    // Expression for ODE right-hand side
    MX z = 1-pow(x(1),2);
    MX rhs = vertcat(z*x(0)-x(1)+2*tanh(p),x(0));
    // ODE declaration with free parameter
    MXDict ode = {{"x",x},{"p",p},{"ode",rhs}};

    // Construct a Function that integrates over 1s
    Function F = integrator("F","cvodes",ode,{{"tf",1}});

    // Control vector
    MX u = MX::sym("u",4,1);

    x = DM(std::vector<double>{0,1});  // Initial state
    for (int k=0;k<4;++k) {
        // Integrate 1s forward in time:
        // call integrator symbolically
        MXDict res = F({{"x0",x},{"p",u(k)}});
        x = res["xf"];
    }

    // NLP declaration
    MXDict nlp = {{"x",u},{"f",dot(u,u)},{"g",x}};

    // Solve using IPOPT
    Function solver = nlpsol("solver","snopt",nlp);
    //Function solver = nlpsol("solver","ipopt",nlp);
    DMDict res = solver(DMDict{{"x0",0.2},{"lbg",0},{"ubg",0}});
}
*/
