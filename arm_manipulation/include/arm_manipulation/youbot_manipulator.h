#ifndef YOUBOT_MANIPULATOR
#define YOUBOT_MANIPULATOR

#include <ros/ros.h>

#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/JointState.h>
#include <arm_kinematics/PoseArray.h>
#include <arm_kinematics/ManipulatorPose.h>
#include <arm_manipulation/MoveLine.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <arm_kinematics/ArmKinematics.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClent;

// Message generation
brics_actuator::JointPositions createArmPositionMsg(const JointValues & jointAngles);
brics_actuator::JointPositions createGripperPositionMsg(double jointValue);

class YoubotManipulator
{
    public:
        YoubotManipulator(ros::NodeHandle & nodeHandle);
        ~YoubotManipulator();

        bool moveArm(const Pose & pose);
        bool moveArm(const JointValues & angles);
        void moveGripper(double jointValue);

        void moveToLineTrajectory(const Pose & startPose, const Pose & endPose);
        void moveArmLoop();

        void initArmTopics();
        void initActionClient(const double aMax, const double vMax, const double timeStep);

    private:
        ros::NodeHandle nh;

        // Kinematics solver
        ArmKinematics solver;

        // Publishers
        ros::Publisher armPublisher;
        ros::Publisher gripperPublisher;

        ros::Subscriber stateSubscriber;

        // Kinematic constansts for trajectory control, lr - youbot_driver rate
        double maxVel, maxAccel, lr, timeStep;
        bool sim;//simulation control

        // Trajectory action client
        ActionClent * trajectoryAC;

        // Service for read point for manipulation to follow trajectory
        ros::ServiceServer trajectoryServer;

        // Service for manipulator to move to angles
        ros::ServiceServer poseServer;

        JointValues stateValues;

        // Callbacs
        bool graspObject(const Pose & p);
        bool putObject(const Pose & p);
        bool goToPose(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res);
        bool trajectoryMove(arm_manipulation::MoveLine::Request & req, arm_manipulation::MoveLine::Response & res);
        void stateCallback(const sensor_msgs::JointStatePtr & msg);
        bool checkAchievementOfPosition(const JointValues & desiredValues);

};
#endif