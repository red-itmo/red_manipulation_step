#ifndef YOUBOT_MANIPULATOR
#define YOUBOT_MANIPULATOR

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/JointState.h>
#include <red_msgs/ArmPoses.h>
#include "std_srvs/Empty.h"//for /SwitchMotorOff service

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
        bool moveArm(const JointValues & angles, const double maxVelocity, const double maxAcceleration);
        void moveGripper(double jointValue);

        bool moveToLineTrajectory(const Pose & startPose, const Pose & endPose, const double mvMaxVel = 0, const double mvMaxAccel = 0);
        void moveToLineTrajectory(const Pose & startPose, const std::vector<Pose> & segmentsPose);
        void moveArmLoop();

        void initArmTopics();
        void setConstraints(const double aMax, const double vMax, const double timeStep);
        bool goToInitAndRelax();
        bool serviceTurnOnMotors(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        bool serviceTurnOffMotors(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

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

        // Trajectory action client
        ActionClent * trajectoryAC;

        // Service for read point for manipulation to follow trajectory
        ros::ServiceServer trajectoryServer;

        // Service for manipulator to move to angles
        ros::ServiceServer poseServer;

        ros::ServiceServer srvRelaxMotorsServer, srvAliveMotorsServer;

        JointValues stateValues;

        // Callbacs
        bool goToPose(red_msgs::ArmPoses::Request & req, red_msgs::ArmPoses::Response & res);
        bool trajectoryMove(red_msgs::ArmPoses::Request & req, red_msgs::ArmPoses::Response & res);
        void stateCallback(const sensor_msgs::JointStatePtr & msg);
        bool checkAchievementOfPosition(const JointValues & desiredValues);
        bool readJV(JointValues & val);

};
#endif