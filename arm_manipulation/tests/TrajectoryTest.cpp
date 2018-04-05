#include <arm_manipulation/youbot_manipulator.h>
#include <trajectory_generator/TrajectoryGenerator.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
// Simple action client at the arm_manipulation/youbot_manipulator.h

int main(int argc, char  ** argv)
{
    ros::init(argc, argv, "trajectory_test");
    ros::NodeHandle nh;

    // Trajectory params
    double timeStep = 0.05, maxVel = 0.05, maxAcc = 0.1;

    nh.param("/trajectory_test/a_m", maxAcc, 0.1);
    if (!nh.hasParam("/trajectory_test/a_m"))
    {
        ROS_WARN("[Arm Manipulation]No param '/trajectory_test/a_m', setting 0.1");
    }
    nh.param("/trajectory_test/v_m", maxVel, 0.05);
    if (!nh.hasParam("/trajectory_test/v_m"))
    {
        ROS_WARN("[Arm Manipulation]No param '/trajectory_test/v_m', setting 0.05");
    }
    nh.param("/trajectory_test/time_step", timeStep, 0.2);
    if (!nh.hasParam("/trajectory_test/time_step"))
    {
        ROS_WARN("[Arm Manipulation]No param '/trajectory_test/time_step', setting 0.2");
    }

    std::vector<Pose> startPoses;
    double xMin = 0.4, xMax = 0.41;
    double yMin = -0.1, yMax = 0.1;
    double step = 0.05;
    Pose p1,p2, p;
    double alpha = 3.1415;

    ROS_INFO_STREAM("[TRJ test] Generate points grid");



    return 0;
}