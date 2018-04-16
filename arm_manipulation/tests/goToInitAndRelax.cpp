#include <arm_manipulation/youbot_manipulator.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "goToInitAndRelax");
    ros::NodeHandle nh;

    YoubotManipulator manipulator(nh);

    ROS_INFO_STREAM("[goToInitAndRelax] Start Work.");
    manipulator.goToInitAndRelax();

    return 0;
}