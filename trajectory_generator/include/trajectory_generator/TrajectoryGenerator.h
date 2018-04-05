#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>
#include <trajectory_msgs/JointTrajectory.h>

class Trajectory {
public:
    Trajectory();
    ~Trajectory();

    //fills velTra, posTra, time and rotTra
    void calculateWorkSpaceTrajectory(const double maxVel, const double maxAccel, const Pose & startPose, const Pose & endPose, const double timeStep);
    void convertWorkSpaceToJointSpace(Pose startPose, Pose endPose, const double timeStep);
    void generateTrajectoryMsg(trajectory_msgs::JointTrajectory & trajectory);
    void exponencialMovingAvarage(std::vector<JointValues> & data, std::vector<JointValues> & movingAvarage, const double alpha);
    bool check_feasible(const Pose startPose, Pose endPose);

    std::vector<JointValues> qTra, qdotTra, qdotdotTra, ema;
    std::vector<Vector3d> posTra, velTra, accTra, rotTra;

    std::vector<double> time;
private:

    double getVel(double time);
    Vector3d getRotVel(double time);
    double t1, t2, t3, maxVel, maxAccel;
    int directionSign;
    Vector3d rotVel;
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator(const double maxVel, const double maxAccel, const double dt);
    ~TrajectoryGenerator();

    std::vector<JointValues> calculateTrajectory(const Pose & startPose, const Pose & endPose);

    void setMaxVelocity(const double maxVel);
    void setMaxAcceleration(const double maxAccel);
    void setTimeStep(const double dt);

    trajectory_msgs::JointTrajectory trajectory;
private:

    double maxVelocity;
    double maxAcceleration;
    double timeStep;

};

#endif