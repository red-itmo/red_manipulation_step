#include <trajectory_generator/TrajectoryGenerator.h>

TrajectoryGenerator::TrajectoryGenerator(const double maxVel, const double maxAccel, const double dt) 
: maxVelocity(maxVel), maxAcceleration(maxAccel), timeStep(dt)
{}

TrajectoryGenerator::~TrajectoryGenerator()
{}

void TrajectoryGenerator::calculateTrajectory(const Pose & startPose, const Pose & endPose)
{
	Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVelocity, maxAcceleration, startPose, endPose, timeStep);
    traj.convertWorkSpaceToJointSpace(timeStep);
    traj.generateTrajectoryMsg(trajectory);
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