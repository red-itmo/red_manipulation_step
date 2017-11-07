#include <arm_manipulation/youbot_manipulator.h>
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "testing_node");
    ros::NodeHandle nh;

    YoubotManipulator manipulator(nh);
    manipulator.initArmTopics();

    // Initial conditions
    Pose pose;
    JointValues angles;
    std::string answer;

    std::cout << "Do you wont to work, condition? (pos, ang, n): "; std::cin >> answer;
    while (nh.ok() && (answer == "pos" || answer == "ang")) {

        if (answer == "pos") {
            std::cout << "Position (x, y, z): "; std::cin >> pose.position(0) >> pose.position(1) >> pose.position(2);
            std::cout << "Angle (phi5, alpha): "; std::cin >> pose.orientation(1) >> pose.orientation(2);

            manipulator.moveArm(pose);
        }
        if (answer == "ang") {
            std::cout << "Joint Angles (q1 - q5): "; std::cin >> angles(0) >> angles(1) >> angles(2) >> angles(3) >> angles(4) >> angles(5);
            manipulator.moveArm(angles);
        }
        std::cout << "------------------------------------------" << std::endl;
    }

    return 0;
}
