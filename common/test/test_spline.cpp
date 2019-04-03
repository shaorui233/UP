#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Utilities/utilities.h"
#include "Utilities/Utilities_print.h"
#include "Utilities/save_file.h"
#include "Utilities/BezierCurve.h"
#include <Configuration.h>

TEST(Spline, BezierCurve_test) {

    constexpr int dim = 3;
    constexpr int num_ctrl_pt = 4;

    BezierCurve<double, dim, num_ctrl_pt> bc;
    double** ctrl_pt = new double * [num_ctrl_pt];

    for(size_t i(0); i<4; ++i){
        ctrl_pt[i] = new double [dim];
   }
    // Initial
    ctrl_pt[0][0] = 0.0;
    ctrl_pt[0][1] = 0.0;
    ctrl_pt[0][2] = 0.4;

    // Initial
    ctrl_pt[1][0] = 0.4;
    ctrl_pt[1][1] = -0.7;
    ctrl_pt[1][2] = 0.5;

    // Initial
    ctrl_pt[2][0] = 2.5;
    ctrl_pt[2][1] = 0.5;
    ctrl_pt[2][2] = 0.7;

    // Final
    ctrl_pt[3][0] = 4.2;
    ctrl_pt[3][1] = -1.0;
    ctrl_pt[3][2] = 0.9;

    double end_time(3.);
    bc.SetParam(ctrl_pt, end_time);

    double curve_pt[dim];
    double curve_vel[dim];

    std::string folder_name = "/common/test/test_data/";
    create_folder(folder_name);
    double t;
    for(size_t i(0); i<2001; ++i){
       t = (double)i * end_time/2000.;
       bc.getCurvePoint(t, curve_pt);
       bc.getCurveVelocity(t, curve_vel);
       saveVector(curve_pt, folder_name, "spline", dim);
       saveVector(curve_vel, folder_name, "spline_vel", dim);
       saveValue(t, folder_name, "time");
    }

    bc.getCurvePoint(0., curve_pt);
    EXPECT_TRUE(fpEqual(curve_pt[0], 0., .0001));
    EXPECT_TRUE(fpEqual(curve_pt[1], 0.0, .0001));
    EXPECT_TRUE(fpEqual(curve_pt[2], 0.4, .0001));

    bc.getCurvePoint(end_time, curve_pt);
    EXPECT_TRUE(fpEqual(curve_pt[0], 4.2, .0001));
    EXPECT_TRUE(fpEqual(curve_pt[1], -1.0, .0001));
    EXPECT_TRUE(fpEqual(curve_pt[2], 0.9, .0001));

    for(size_t i(0); i<num_ctrl_pt; ++i){
    delete [] ctrl_pt[i];
    }
    delete [] ctrl_pt;
}

