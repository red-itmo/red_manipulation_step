#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing_node");
    ros::NodeHandle nh;

    if (argc == 1) {
        std::cout << "Usage: rosrun arm_manipulation move_gripper [open/close]" << std::endl;
        return 1;
    }

    std::string str(argv[1]);

    YoubotManipulator manipulator(nh);
    ros::Duration(2).sleep();

    if (str == "open") manipulator.moveGripper(0.0115);
    else if (str == "close") manipulator.moveGripper(0.0);
    else std::cout << "Not valid parameter\nUsage: rosrun arm_manipulation move_gripper [open/close]" << std::endl;

    return 0;
}
