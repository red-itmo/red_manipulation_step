#include <manipulation_control_node/ManipulationControlNode.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "control_manipulation_node");
    ros::NodeHandle nh;

    ManipulationControlNode controlNode(nh);
    ROS_INFO_STREAM("Start work!");
    controlNode.start();

    return 0;
}