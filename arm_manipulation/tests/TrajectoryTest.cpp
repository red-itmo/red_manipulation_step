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
    ROS_INFO("Starting trajectory_test...");

    // Trajectory params
    double timeStep, maxVel, maxAcc;
    std::vector<double> startPosition, endPosition;

    bool reading = nh.getParam("/trajectory_test/startPosition", startPosition);

    nh.getParam("/trajectory_test/endPosition", endPosition);
    nh.getParam("/trajectory_test/startPosition", startPosition);
    nh.param("/trajectory_test/a_m", maxAcc, 0.1);
    nh.param("/trajectory_test/v_m", maxVel, 0.05);
    nh.param("/trajectory_test/time_step", timeStep, 0.2);

    if (!reading) {
        ROS_FATAL_STREAM("[WST test] Parameters launch file is not found.");
        return 1;
    } else {
        ROS_INFO_STREAM("[WST test] Start position ("
                        << startPosition[0]
                        << ", " << startPosition[1]
                        << ", " << startPosition[2]
                        << ", " << startPosition[3]
                        << ", " << startPosition[4] << ")");

        ROS_INFO_STREAM("[WST test] End position ("
                        << endPosition[0]
                        << ", " << endPosition[1]
                        << ", " << endPosition[2]
                        << ", " << endPosition[3]
                        << ", " << endPosition[4] << ")");
        ROS_INFO_STREAM("[WST test] Max vel.: " << maxVel);
        ROS_INFO_STREAM("[WST test] Max accel.: " << maxAcc);
        ROS_INFO_STREAM("[WST test] Time step: " << timeStep);
    }

    // Acception step
    std::string acception = "y";
    std::cout << "Start trajectory test? (y, n)"; std::cin >> acception;
    if (acception != "y") return 1;

    YoubotManipulator youbotManipulator(nh);
    youbotManipulator.setConstraints(maxAcc, maxVel, timeStep);

    Pose startPose, endPose;

    startPose.position(0) = startPosition[0];
    startPose.position(1) = startPosition[1];
    startPose.position(2) = startPosition[2];
    startPose.orientation(0) = startPosition[3];
    startPose.orientation(1) = startPosition[4];

    endPose.position(0) = endPosition[0];
    endPose.position(1) = endPosition[1];
    endPose.position(2) = endPosition[2];
    endPose.orientation(0) = endPosition[3];
    endPose.orientation(1) = endPosition[4];

    youbotManipulator.moveToLineTrajectory(startPose, endPose);

    ROS_INFO("Ready to receive another service command!");
    youbotManipulator.moveArmLoop();

}