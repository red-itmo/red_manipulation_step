#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_generator/TrajectoryGenerator.h>
#include <brics_actuator/JointPositions.h>
#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_ros_simple_trajectory");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/velocity_joint_trajecotry", true);
    ros::Publisher armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending trajectory.");

    // setup trajectory message
    control_msgs::FollowJointTrajectoryGoal msg;
    JointValues startAngles;
    trajectory_msgs::JointTrajectory lineTrajectory;
    brics_actuator::JointPositions jointPositions;

    // START GENERATE trajectory ---------------------------
    double lr = 100;

    double maxVel = 0.05;
    double maxAccel = 0.1;
    double startVel = 0;
    double endVel = 0;
    double alpha = 0;

    YoubotManipulator manipulator(n);
    ArmKinematics solver;
    JointValues jointAngles;

    Pose startPos;
    startPos.position(0) = -0.25;
    startPos.position(1) = 0.1;
    startPos.position(2) = -0.037 + 0.05;
    startPos.orientation(0) = 0;
    startPos.orientation(1) = 0;
    startPos.orientation(2) = -3.1415;

    if (!solver.solveFullyIK(startPos, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPos.position(0) << ", " << startPos.position(1)  << ", " << startPos.position(2));
        return 1;
    }
    makeYoubotArmOffsets(jointAngles);
    ROS_INFO_STREAM("Angles: (" << jointAngles(0)
    << ", " << jointAngles(1)
    << ", " << jointAngles(2)
    << ", " << jointAngles(3)
    << ", " << jointAngles(4) << ")");

    Pose endPos = startPos;
    endPos.position(2) = -0.037;
    if (!solver.solveFullyIK(endPos, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (endPos): " << endPos.position(0) << ", " << endPos.position(1)  << ", " << endPos.position(2));
        return 1;
    }

    TrajectoryGenerator gen(maxVel, maxAccel, 1/lr);
    gen.calculateTrajectory(startPos, endPos);

    // END GENERATE trajectory ---------------------------
    ROS_INFO_STREAM("[Arm Manipulation] Move to initial position.");
    for (size_t i = 0; i < 5; ++i) {
        startAngles(i) = gen.trajectory.points[0].positions[i];
    }
    jointPositions = createArmPositionMsg(startAngles);
    armPublisher.publish(jointPositions);
    ros::Duration(2).sleep();

    // for (size_t p = 1; p < lineTrajectory.points.size(); ++p) {
    //     for (int i = 2; i < 5; ++i) {
    //         lineTrajectory.points[p].positions[i] = lineTrajectory.points[0].positions[i];
    //         lineTrajectory.points[p].velocities[i] = 0;
    //         lineTrajectory.points[p].accelerations[i] = 0;
    //     }
    //     ROS_INFO_STREAM("Angles: (" << lineTrajectory.points[p].positions[0] 
    //         << ", " << lineTrajectory.points[p].positions[1] 
    //         << ", " << lineTrajectory.points[p].positions[2]
    //         << ", " << lineTrajectory.points[p].positions[3]
    //         << ", " << lineTrajectory.points[p].positions[4] << ")");
    // }

    std::string check;
    std::cout << "Start?" << std::endl;
    std::cin >> check;
    if (check != "yes") return 0;

    msg.trajectory = gen.trajectory;
    
    // fill message header and sent it out
    msg.trajectory.header.frame_id = "arm_link_0";
    msg.trajectory.header.stamp = ros::Time::now();

    ROS_INFO("Start trajectory execute!");
    ac.sendGoal(msg);

    // wait for reply that action was completed (or cancel after 10 sec)
    ac.waitForResult(ros::Duration(10));
    
    return 0;
}