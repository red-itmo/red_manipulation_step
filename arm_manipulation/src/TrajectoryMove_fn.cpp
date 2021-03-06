#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

bool YoubotManipulator::trajectoryMove(red_msgs::ArmPoses::Request & req, red_msgs::ArmPoses::Response & res)
{
    Pose startPose, endPose;

    if (req.poses.size() != 2) {
        ROS_ERROR("Size of input array is NOT VALID");
        return false;
    }

    startPose.position(0) = req.poses[0].x;
    startPose.position(1) = req.poses[0].y;
    startPose.position(2) = req.poses[0].z;
    startPose.orientation(0) = req.poses[0].theta;
    startPose.orientation(1) = req.poses[0].psi;

    endPose.position(0) = req.poses[1].x;
    endPose.position(1) = req.poses[1].y;
    endPose.position(2) = req.poses[1].z;
    endPose.orientation(0) = req.poses[1].theta;
    endPose.orientation(1) = req.poses[1].psi;

    if (!moveToLineTrajectory(startPose, endPose, req.vel, req.accel))
        return false;

    res.error = 0;
    return true;
}

bool YoubotManipulator::moveToLineTrajectory(const Pose & startPose, const Pose & endPose, const double mvMaxVel, const double mvMaxAccel)
{
    JointValues startAngles;
    brics_actuator::JointPositions jointPositions;
    double startVel = 0;
    double endVel = 0;

    ROS_INFO_STREAM("[Arm Manipulation,moveToLineTrajectory] Max Vel: " << maxVel << " Max Accel: " << maxAccel <<" TimeStep: " << timeStep);
    TrajectoryGenerator gen(maxVel, maxAccel, timeStep);
    ROS_INFO("Calculating trajectory...");
    std::vector<JointValues> rotationsTrajectory=gen.calculateTrajectory(startPose, endPose);

    //if error occured during trajectory generation
    if(rotationsTrajectory.size()==0)
    {
        return false;
    }

    ROS_INFO_STREAM("[Arm Manipulation] Move to initial position.");
    for (size_t i = 0; i < 5; ++i) {
        startAngles(i) = gen.trajectory.points[0].positions[i];
    }

    ROS_INFO("Going to initial position...");

    // std::cout<<"Generated trj angles:\n";
    // for (size_t j = 0; j < gen.trajectory.points.size(); ++j) {
    //     for (size_t i = 0; i < 5; ++i)
    //          std::cout<<gen.trajectory.points[j].positions[i]<<" ";
    //     std::cout<<"\n";
    //     }

    if (mvMaxVel == 0 || mvMaxAccel == 0)
        moveArm(startAngles);
    else
        moveArm(startAngles, mvMaxVel, mvMaxAccel);
    ros::Duration(1).sleep();

    // std::string acception = "y";
    // std::cout << "Proceed? (y, n)"; std::cin >> acception;
    // if (acception != "y") return;

    if (!gen.trajectory.points.empty())
    {
        ROS_INFO("Waiting for server...");
        trajectoryAC->waitForServer();
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
    return true;
}

void YoubotManipulator::moveToLineTrajectory(const Pose & startPose, const std::vector<Pose> & segmentsPose)
{
    JointValues startAngles;
    brics_actuator::JointPositions jointPositions;
    double startVel = 0;
    double endVel = 0;

    ROS_INFO_STREAM("[Arm Manipulation,moveToLineTrajectory] Max Vel: " << maxVel << " Max Accel: " << maxAccel <<" TimeStep: " << timeStep);
    TrajectoryGenerator gen(maxVel, maxAccel, timeStep);
    ROS_INFO("Calculating trajectory...");
    std::vector<JointValues> rotationsTrajectory=gen.calculateTrajectory(startPose, segmentsPose);

    //if error occured during trajectory generation
    if(rotationsTrajectory.size()==0)
    {
        return;
    }

    ROS_INFO_STREAM("[Arm Manipulation] Move to initial position.");
    for (size_t i = 0; i < 5; ++i) {
        startAngles(i) = gen.trajectory.points[0].positions[i];
    }

    ROS_INFO("Going to initial position...");

    // std::cout<<"Generated trj angles:\n";
    // for (size_t j = 0; j < gen.trajectory.points.size(); ++j) {
    //     for (size_t i = 0; i < 5; ++i)
    //          std::cout<<gen.trajectory.points[j].positions[i]<<" ";
    //     std::cout<<"\n";
    //     }

    moveArm(startAngles);
    // ros::Duration(2).sleep();

    // std::string acception = "y";
    // std::cout << "Proceed? (y, n)"; std::cin >> acception;
    // if (acception != "y") return;

    if (!gen.trajectory.points.empty())
    {
        ROS_INFO("Waiting for server...");
        trajectoryAC->waitForServer();
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