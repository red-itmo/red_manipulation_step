#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

bool YoubotManipulator::trajectoryMove(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res)
{
    Pose p;
    p.position(0) = req.pose.position[0];
    p.position(1) = req.pose.position[1];
    p.position(2) = req.pose.position[2];
    p.orientation(0) = req.pose.orientation[0];
    p.orientation(1) = req.pose.orientation[1];
    p.orientation(2) = req.pose.orientation[2];

    if (req.task == 1) {
        res.feasible = graspObject(p);
        return true;
    }
    else if (req.task == 2) {
        res.feasible = putObject(p);
        return true;
    }
    else
        ROS_WARN("task: 1 or 2");
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
     
    ros::Duration(0.5).sleep();
     
    ROS_INFO("Publishing trajectory...");
    for(int i=0;i<rotationsTrajectory.size();i++){
        armPublisher.publish(createArmPositionMsg(rotationsTrajectory[i]));
        ros::Duration(0.01).sleep();
    }

    //temporarily disabled to be able to launch simulation
    if (!gen.trajectory.points.empty() && !sim)
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