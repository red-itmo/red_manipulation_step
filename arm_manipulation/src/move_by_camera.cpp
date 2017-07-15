#include <arm_manipulation/youbot_manipulator.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "move_by_camera");
    ros::NodeHandle nh;

    YoubotManipulator manipulator(nh);

    ROS_INFO_STREAM("[Move by camera] Start Work.");
    manipulator.moveArmLoop();

    return 0;
}
