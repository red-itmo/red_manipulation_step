#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

bool YoubotManipulator::trajectoryMove(arm_manipulation::MoveLine::Request & req, arm_manipulation::MoveLine::Response & res)
{
    Pose startPose;
    startPose.position(0) = req.startPose.position[0];
    startPose.position(1) = req.startPose.position[1];
    startPose.position(2) = req.startPose.position[2];
    startPose.orientation(0) = req.startPose.orientation[0];
    startPose.orientation(1) = req.startPose.orientation[1];
    startPose.orientation(2) = req.startPose.orientation[2];

    Pose endPose;
    endPose.position(0) = req.endPose.position[0];
    endPose.position(1) = req.endPose.position[1];
    endPose.position(2) = req.endPose.position[2];
    endPose.orientation(0) = req.endPose.orientation[0];
    endPose.orientation(1) = req.endPose.orientation[1];
    endPose.orientation(2) = req.endPose.orientation[2];

    moveToLineTrajectory(startPose, endPose);
}

void YoubotManipulator::moveToLineTrajectory(const Pose & startPose, const Pose & endPose)
{
    JointValues startAngles;
    brics_actuator::JointPositions jointPositions;
    double startVel = 0;
    double endVel = 0;

    ROS_INFO_STREAM("[Arm Manipulation] Max Vel: " << maxVel << " Max Accel: " << maxAccel <<" TimeStep: " << timeStep);
    TrajectoryGenerator gen(maxVel, maxAccel, timeStep);
    ROS_INFO("Calculating trajectory...");
    std::vector<JointValues> rotationsTrajectory=gen.calculateTrajectory(startPose, endPose);

    ROS_INFO_STREAM("[Arm Manipulation] Move to initial position.");
    for (size_t i = 0; i < 5; ++i) {
        startAngles(i) = gen.trajectory.points[0].positions[i];
    }
    jointPositions = createArmPositionMsg(startAngles);

    ROS_INFO("Going to initial position...");
    armPublisher.publish(jointPositions);
     
    ros::Duration(2).sleep();
    
    if (!gen.trajectory.points.empty())
    {
        ROS_INFO("Waiting for server...");
        trajectoryAC->waitForServer(); // Will wait for infinite time
        ROS_INFO("[Arm Manipulation] Action server started, sending goal.");

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = gen.trajectory;

        trajectoryAC->sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = trajectoryAC->waitForResult(ros::Duration(60.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = trajectoryAC->getState();
            ROS_INFO("[Arm Manipulation] Action finished: %s", state.toString().c_str());
        } else ROS_ERROR("[Arm Manipulation] Action did not finish before the time out.");
    }
}