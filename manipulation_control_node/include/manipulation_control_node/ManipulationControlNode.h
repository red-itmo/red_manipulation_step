#ifndef MANIPULATION_CONTROL_NODE
#define MANIPULATION_CONTROL_NODE

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <utility>

class ManipulationControlNode 
{
    public:
        ManipulationControlNode(ros::NodeHandle n);
        ~ManipulationControlNode();

        void start();

    private:
        bool execute(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
        size_t checkContainerContents(std::pair<std::vector<bool>, std::vector<Pose>> & container);

        double cameraOffsetX, cameraOffsetY, cameraOffsetZ;
        JointValues currentJointAngles;
        ArmKinematics solver;

        ros::NodeHandle nh;

        ros::ServiceClient getListObjectsClient;

        ros::ServiceClient graspPosesClient;

        ros::ServiceClient moveToPoseClient;

        ros::ServiceServer tableServer;

        std::pair<std::vector<bool>, std::vector<Pose>> objectContainer;
};
#endif