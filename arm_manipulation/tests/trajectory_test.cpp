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
    double timeStep = 0.01, maxVel = 0.05, maxAcc = 0.1;

    bool reading = nh.getParam("/trajectory_test/a_m", maxAcc);
    nh.getParam("/trajectory_test/v_m", maxVel);
    nh.getParam("/trajectory_test/time_step", timeStep);    // TODO

    if (!reading) {
        ROS_FATAL_STREAM("[TRJ test] Parameters of launch file is not found.");
        return 1;
    }

    std::vector<Pose> startPoses;
    double xMin = 0.4, xMax = 0.41;
    double yMin = -0.1, yMax = 0.1;
    double step = 0.01; Pose p;
    double alpha = 3.1415;

    ROS_INFO_STREAM("[TRJ test] Generate points grid");

    for (double x = xMin; x <= xMax; x += step) {
        for (double y = yMin; y <= yMax; y += step) {
            ROS_INFO_STREAM("(x, y) -- (" << x << ", " << y << ")");
            p.position(0) = x;
            p.position(1) = y;
            p.position(2) = 0;
            p.orientation(2) = alpha;

            startPoses.push_back(p);
        }
    }

    // p.position(0) = 0.37;
    // p.position(1) = -0.08;
    // p.position(2) = 0;
    // p.orientation(2) = alpha;
    // startPoses.push_back(p);

    ArmKinematics solver;
    YoubotManipulator manipulator(nh);
    manipulator.initArmTopics();
    manipulator.initActionClient(maxAcc, maxVel);

    // Acception step
    std::string acception;
    std::cout << "Start trajectory test? (y, n)"; std::cin >> acception;
    if (acception != "y") return 1;

    // Execution step
    Pose startPose, endPose;
    JointValues angles;
    int i = 0;
    double alpha1, alpha2;

    while (nh.ok() && (i < startPoses.size())) {

        ROS_INFO_STREAM("[TRJ test] Point (" << startPoses[i].position(0) 
            << ", " << startPoses[i].position(1) 
            << ", " << startPoses[i].position(2) << ")");

        startPose = startPoses[i];
        if (!solver.solveFullyIK(startPose, angles)) {
            ROS_ERROR_STREAM("Solution start pose not found!");
            return 1;
        }
        alpha1 = angles(1) + angles(2) + angles(3);

        endPose = startPose;
        endPose.position(2) = -0.05;
        // endPose.orientation(2) = alpha1;
        endPose.orientation(2) = alpha;
        if (!solver.solveFullyIK(endPose, angles)) {
            ROS_ERROR_STREAM("Solution end pose not found!");
            return 1;
        }
        alpha2 = angles(1) + angles(2) + angles(3);

        // if (alpha1 != alpha2) {
        //     startPose.orientation(2) = alpha2;
        //     if (!solver.solveFullyIK(startPose, angles)) {
        //         ROS_ERROR_STREAM("Solution start pose not found!");
        //         return 1;
        //     }
        //     alpha1 = angles(1) + angles(2) + angles(3);
        //     ROS_INFO_STREAM("[WST test] Start alpha: " << alpha1);
        // }   

        manipulator.moveArm(startPose);
        ros::Duration(2).sleep();
        manipulator.moveToLineTrajectory(startPose, endPose);

        manipulator.moveArm(startPose);
        ros::Duration(2).sleep();

        ++i;
    }
    ROS_INFO_STREAM("END");

    return 0;
}