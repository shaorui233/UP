#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Utilities/utilities.h"
#include "../third-party/Goldfarb_Optimizer/QuadProg++.hh"

TEST(Goldfarb_Optimizer, Goldfarb_opt_test) {
    // problem:
    // >> H = [4 1; 1 2]; f = [1;1]; A = [1 1; 1 0; 0 1; -1 -1; -1 0; 0 -1]; b = [1 .7 .7 -1 0 0]';
    // >> x = quadprog(H,f,A,b)
    // x =
    //
    //    0.3000
    //    0.7000

    // hessian
    GMatr<double> G(2,2);
    G[0][0] = 4.0; G[0][1] = 1.;
    G[1][0] = 1.;  G[1][1] = 2.;

    GVect<double> g0(2);
    g0[0] = 1.;
    g0[1] = 1.;
    //c_float P_x[4] = {4.00, 1.00, 1.00, 2.00, };
    //c_int P_nnz = 4;
    //c_int P_i[4] = {0, 1, 0, 1, };
    //c_int P_p[3] = {0, 2, 4, };

    // gradient
    //c_float q[2] = {1.00, 1.00, };

    // constraint grad
    //c_float A_x[4] = {1.00, 1.00, 1.00, 1.00, };
    //c_int A_nnz = 4;
    //c_int A_i[4] = {0, 1, 0, 2, };
    //c_int A_p[3] = {0, 2, 4, };

    GMatr<double> CE(2, 1);
    CE[0][0] = 1.;
    CE[1][0] = 1.;
    GVect<double> ce0(1);
    ce0[0] = -1.0;

    GMatr<double> CI(2, 4);
    CI[0][0] = 1.;
    CI[1][1] = 1.;
    CI[0][2] = -1.;
    CI[1][3] = -1.;

    GVect<double> ci0(4);
    ci0[0] = 0.;
    ci0[1] = 0.;
    ci0[2] = 0.7;
    ci0[3] = 0.7;
    // bounds
    //c_float l[3] = {1.00, 0.00, 0.00, };
    //c_float u[3] = {1.00, 0.70, 0.70, };
    //c_int n = 2; // number of vars
    //c_int m = 3; // number of constraints

    // osqp things
    //GoldfarbSettings* settings = (GoldfarbSettings*)malloc(sizeof(GoldfarbSettings));
    //GoldfarbWorkspace* workspace;
    //GoldfarbData* data = (GoldfarbData*)malloc(sizeof(GoldfarbData));
    //data->n = n;
    //data->m = m;
    //data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    //data->q = q;
    //data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    //data->l = l;
    //data->u = u;

    GVect<double> x;
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    printf("cost: %f\n", f);
    printf("solution: %6.3f, %6.3f\n", x[0], x[1]);

    EXPECT_TRUE(fpEqual(x[0], .3, .0001));
    EXPECT_TRUE(fpEqual(x[1], .7, .0001));

}

