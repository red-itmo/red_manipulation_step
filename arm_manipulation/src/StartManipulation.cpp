#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle nh;
    double a_max = 0.1, v_max = 0.05, t_step = 0.05;

    ROS_INFO("[M Node] Start Manipulation Node");

    YoubotManipulator manipulator(nh);
    ros::Duration(2).sleep();

    manipulator.setConstraints(0.1, 0.05, 0.05);
    manipulator.moveArmLoop();

    return 0;
}