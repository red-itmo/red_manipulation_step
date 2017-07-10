#ifndef YOUBOT_MANIPULATOR
#define YOUBOT_MANIPULATOR

#include <ros/ros.h>

#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/JointState.h>
#include <arm_kinematics/PoseArray.h>
#include <arm_kinematics/ManipulatorPose.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <arm_kinematics/ArmKinematics.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClent;

// Message generation
brics_actuator::JointPositions createArmPositionMsg(JointValues jointAngles);
brics_actuator::JointPositions createGripperPositionMsg(double jointValue);

class YoubotManipulator
{
    public:
        YoubotManipulator(ros::NodeHandle & nodeHandle);
        ~YoubotManipulator();

        void moveArm(const Pose & pose);
        void moveGripper(double jointValue);

        void moveToLineTrajectory(const Pose & startPose, const Pose & endPose);
        void moveArmLoop();

    private:
        ros::NodeHandle nh;

        // Kinematics solver
        ArmKinematics solver;

        // Publishers
        ros::Publisher armPublisher;
        ros::Publisher gripperPublisher;

        // Kinematic constansts for trajectory control, lr - youbot_driver rate
        double maxVel, maxAccel, lr;

        // Trajectory action client
        ActionClent * trajectoryAC;

        // Service for read point for manipulation to follow trajectory
        ros::ServiceServer trajectoryServer;

        // Service for manipulator to move to angles
        ros::ServiceServer poseServer;

        // Camera offset
        double cameraOffsetX, cameraOffsetY, cameraOffsetZ;

        // Callbacs
        bool graspObject(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res);
        bool goToPose(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res);
};
#endif
