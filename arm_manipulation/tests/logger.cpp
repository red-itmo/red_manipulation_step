#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <fstream>

std::ofstream logFile;
ros::Time startTime;
ros::Duration t;

void callback(const sensor_msgs::JointState & msg) 
{
    if (msg.position.size() == 7) {
        t = ros::Time::now() - startTime;
                   // arm_joint_2             // arm_joint_3             // arm_joint_3
        logFile << msg.position[1] << "\t" << msg.position[2] << "\t" << msg.position[3] << "\t" << t << "\n";
        // ROS_INFO_STREAM("Value" << msg.position[1] << "\t" << msg.position[2] << "\t" << msg.position[3] << "\t" << t);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "logger");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Subscribe to topic: /joint_states");
    ros::Subscriber jointStateSubsciber = nh.subscribe("/joint_states", 1, callback);
    ros::Duration(2).sleep();

    std::string filename = "/home/senex/velocity_control/src/JointValue1.log";
    ROS_INFO_STREAM("Create file " << filename);
    logFile.open(filename.c_str());

    ROS_INFO_STREAM("Start write ot file.");
    startTime = ros::Time::now();
    while(nh.ok()) {
        ros::spin();
    }

    logFile.close();
}