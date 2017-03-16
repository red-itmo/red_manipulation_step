#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing_node");
    ros::NodeHandle nh;

    YoubotManipulator manipulator(nh);

    // Initial conditions
    Pose pose;

    while (nh.ok()) {

        std::cout << "Position (x, y, z): "; std::cin >> pose.position(0) >> pose.position(1) >> pose.position(2);
        std::cout << "Angle (phi5, alpha): "; std::cin >> pose.orientation(1) >> pose.orientation(2);

        manipulator.moveArm(pose);
        std::cout << "------------------------------------------" << std::endl;
    }

    return 0;
}
