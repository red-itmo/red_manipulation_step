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
    //if trajectory in configuration space isn't the same that of work space
    //due to error while converting between workspaces
    if(traj.qTra.size()==0 || traj.qTra.size()!=traj.posTra.size())
    {
        ROS_FATAL_STREAM("Error in trajectory!");
        std::vector<JointValues> temp;
        //return empty array
        return temp;
    }
    traj.generateTrajectoryMsg(trajectory);
    return traj.qTra;
}

std::vector<JointValues> TrajectoryGenerator::calculateTrajectory(const Pose & startPose, const std::vector<Pose> & segmentsPose)
{
    Trajectory traj;
    traj.mstraj(maxVelocity, timeStep, maxAcceleration, startPose, segmentsPose);
    Pose endPose;   //isn't used
    traj.convertWorkSpaceToJointSpace(startPose, endPose, timeStep);
    //if trajectory in configuration space isn't the same that of work space
    //due to error while converting between workspaces
    if(traj.qTra.size()==0 || traj.qTra.size()!=traj.posTra.size())
    {
        ROS_FATAL_STREAM("Error in trajectory!");
        std::vector<JointValues> temp;
        //return empty array
        return temp;
    }
    traj.generateTrajectoryMsg(trajectory);
    return traj.qTra;
}

std::vector<JointValues> TrajectoryGenerator::calculateTrajectory(const JointValues & startAng, const JointValues & endAng)
{
    Trajectory traj;
    //converting double to vector
    JointValues accel;
    accel.setAll(maxAcceleration);
    JointValues vel;
    vel.setAll(maxVelocity);

    traj.ConfSpaceTrj(startAng,  endAng,  accel,  vel,  timeStep);
    //if error in trajectory configuration space
    if(traj.qTra.size()==0)
    {
        ROS_FATAL_STREAM("Error in trajectory!");
        std::vector<JointValues> temp;
        //return empty array
        return temp;
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