#include <trajectory_generator/TrajectoryGenerator.h>

TrajectoryGenerator::TrajectoryGenerator(const double maxVel, const double maxAccel, const double dt)
: maxVelocity(maxVel), maxAcceleration(maxAccel), timeStep(dt)
{}

TrajectoryGenerator::~TrajectoryGenerator()
{}

std::vector<JointValues> TrajectoryGenerator::calculateTrajectory(const Pose & startPose, const Pose & endPose)
{
	Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVelocity, maxAcceleration, startPose, endPose, timeStep);
    traj.convertWorkSpaceToJointSpace(startPose, endPose, timeStep);
    if(traj.qTra.size()==0)
    {
        ROS_FATAL_STREAM("Error in trajectory!");
        return traj.qTra;
    }
    traj.generateTrajectoryMsg(trajectory);
    return traj.qTra;
}

void TrajectoryGenerator::setMaxVelocity(const double maxVel)
{
	maxVelocity = maxVel;
}

void TrajectoryGenerator::setMaxAcceleration(const double maxAccel)
{
	maxAcceleration = maxAccel;
}

void TrajectoryGenerator::setTimeStep(const double dt)
{
	timeStep = dt;
}
