#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Utilities/utilities.h"
#include <nlopt.hpp>
#include <math.h>

using namespace nlopt;

double myfunc(unsigned n, const double *x, 
        double *grad, void *my_func_data)
{
    (void)n;
    (void)my_func_data;
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    (void)n;
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
}

TEST(nlopt, nlopt_test){
    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL;
    lb[1] = 0; /* lower bounds */

    nlopt::opt* test = new nlopt::opt(nlopt::LD_MMA, 2);/* algorithm and dimensionality */

    test->set_lower_bounds(lb);
    test->set_min_objective(myfunc, NULL);

    my_constraint_data data[2] = { {2,0}, {-1,1} }; 
    test->add_inequality_constraint(myconstraint, &data[0], 1e-8);
    test->add_inequality_constraint(myconstraint, &data[1], 1e-8);
    test->set_xtol_rel(1e-4);

    std::vector<double> x(2);
    x = { 1.234, 5.678 };  /* `*`some` `initial` `guess`*` */
    double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
    if (test->optimize(x, minf) < 0) {
        printf("nlopt failed!\n");
    }
    else {
        printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
    } 
    delete test;
    EXPECT_TRUE(fpEqual(x[0], 0.333334, .0001));
    EXPECT_TRUE(fpEqual(x[1], 0.296296, .0001));
    EXPECT_TRUE(fpEqual(minf, 0.544330847, 0.0001));
}
