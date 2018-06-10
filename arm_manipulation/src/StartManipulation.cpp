#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle nh;
    double a_max = 0.1, v_max = 0.05, t_step = 0.008;

    ROS_INFO("[M Node] Start Manipulation Node");

    YoubotManipulator manipulator(nh);
    ros::Duration(2).sleep();

    manipulator.setConstraints(a_max, v_max, t_step);
    manipulator.moveArmLoop();

    return 0;
}