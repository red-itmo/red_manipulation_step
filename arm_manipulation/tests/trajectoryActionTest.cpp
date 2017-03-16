#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_ros_simple_trajectory");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/velocity_joint_trajecotry", true);
    ros::Publisher armPublisher = n.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending trajectory.");

    // setup trajectory message
    control_msgs::FollowJointTrajectoryGoal msg;
    JointValues startAngles;
    brics_actuator::JointPositions startJntPos;

    // some arbitrary points of a not too useful trajectory
    
    const double dt = 0.01;
    const double vel = 1;
    JointValues jntVel;
    jntVel(3) = 1;
    JointValues jntAccel;
    jntAccel(3) = 0.5;
    int nPoints = (1.3 - 0.3) / (vel*dt) + 2;
    double values[nPoints][5];

    for (int i = 0; i < nPoints; ++i) {
        values[i][0] = 1.5;
        values[i][1] = 0.2;
        values[i][2] = -0.3;
        values[i][3] = 0.3 + jntVel(3)*dt*i;
        values[i][4] = 1.5;
        ROS_INFO_STREAM("Values: " << values[i][0] << " " << values[i][1] << " " << values[i][2] << " " << values[i][3] << " " << values[i][4]);
    }

    startAngles(0) = values[0][0];
    startAngles(1) = values[0][1];
    startAngles(2) = values[0][2];
    startAngles(3) = values[0][3];
    startAngles(4) = values[0][4];
    ROS_INFO_STREAM("start Val: " << values[0][0] << " " << values[0][1] << " " << values[0][2] << " " << values[0][3] << " " << values[0][4]);
    startJntPos = createArmPositionMsg(startAngles);
    armPublisher.publish(startJntPos);
    ros::Duration(1).sleep();

    std::string check;
    std::cout << "Start?" << std::endl;
    std::cin >> check;
    if (check != "yes") return 0; 

    // set values for all points of trajectory
    for (int p = 0; p < nPoints; p++) { // iterate over all points
        trajectory_msgs::JointTrajectoryPoint point;
        for (int i = 0; i < 5; i++) { // 5 DOF
            point.positions.push_back(values[p][i]);
            if (i == 3 && (p == 0 || p == nPoints - 1)) point.velocities.push_back(0);
            else point.velocities.push_back(jntVel(i));
            point.accelerations.push_back(0);
        }
        point.time_from_start = ros::Duration(dt*p);
        msg.trajectory.points.push_back(point);
    }

    // set joint names
    for (int i = 0; i < 5; i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        msg.trajectory.joint_names.push_back(jointName.str());
    }
    
    // fill message header and sent it out
    msg.trajectory.header.frame_id = "arm_link_0";
    msg.trajectory.header.stamp = ros::Time::now();
    ac.sendGoal(msg);

    // wait for reply that action was completed (or cancel after 10 sec)
    ac.waitForResult(ros::Duration(10));
    
    return 0;
}