/*! @file test_trot_path
 *  @brief Test orientation functions
 *
 * Test the orientation related functions in orientation_tools.h
 * Does not check any spatial stuff
 */



#include "Math/orientation_tools.h"
#include "cppTypes.h"
#include "Utilities/utilities.h"
#include "Utilities/Utilities_print.h"
#include "../robot/WBC_States/PlannedTrot/Planner/Path.hpp"
#include "Dynamics/spatial.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"


using namespace std;
using namespace spatial;

TEST(Path, foot_location_compuation){

    Path<double> _path;
    Vec3<double> shoulder, body_pos, body_vel, body_ang_vel, foot_loc;
    Vec3<double> ref; ref.setZero();

    shoulder.setZero();
    body_pos.setZero();
    body_vel.setZero();
    body_ang_vel.setZero();

    Vec3<double> body_ori_rpy;
    Mat3<double> Rot_trans, Rot;
    double step_time = 0.25;
    // Test 1
    body_ori_rpy << 0.3, 0.1, 0.5;
    Rot_trans = ori::rpyToRotMat(body_ori_rpy);
    Rot = Rot_trans.transpose();

    shoulder << 0.25, 0.17, 0 ;
    body_pos << 1.2, 0.3, 0.3;
    body_vel << 0.2, 0.1, 0.05;
    body_ang_vel << 0.1, 0.3, 0.2;
    ref<<1.3643, 0.5789, 0.0; 

    _path.computeFootLoc(Rot, shoulder, step_time, body_pos, body_vel, body_ang_vel, foot_loc);
    //pretty_print(foot_loc, std::cout, "foot loc");
    EXPECT_TRUE(almostEqual(foot_loc, ref, 0.0005));
    
    // Test 2
    body_ori_rpy << 0.3, 0.1, 0.5;
    Rot_trans = ori::rpyToRotMat(body_ori_rpy);
    Rot = Rot_trans.transpose();

    shoulder << 0.25, -0.17, 0 ;
    body_pos << 1.2, 0.3, 0.4;
    body_vel << 0.2, 0.1, 0.02;
    body_ang_vel << 0.1, 0.3, 0.2;
    ref << 1.515, 0.2939, 0.0;

    _path.computeFootLoc(Rot, shoulder, step_time, body_pos, body_vel, body_ang_vel, foot_loc);
    //pretty_print(foot_loc, std::cout, "foot loc");
    EXPECT_TRUE(almostEqual(foot_loc, ref, 0.0005));
}
