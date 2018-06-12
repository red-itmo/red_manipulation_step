#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>
#include <trajectory_msgs/JointTrajectory.h>

#define DEBUG false;
#define MAX_VEL 5

//Class for jointspace trajectory generation
class VelFunc
{

public:
    VelFunc(double maxAccel = 0, double maxVel = 0)
    : ddq_m(maxAccel), dq_m(maxVel), valid(false)
    {}

    void setParams(double maxAccel, double maxVel)
    {
        ddq_m = maxAccel; dq_m = maxVel;
    }

    bool setPose(double initialPos, double endPos, double minPoseDiff)
    {
        q_i = initialPos;
        q_e = endPos;

        q_diff = std::abs(q_e - q_i);
        min_q_diff = minPoseDiff;
        diraction = (q_e > q_i) - (q_e < q_i);
        t0 = 5*minPoseDiff;
        calculateParameters();

        if (!checkTime()) {
            std::cout << "Error: t2 < t1" << std::endl;
            std::cout << "Try correct max vel: ";
            if(maxVelCorrection(q_diff))
                std::cout << dq_m << std::endl;
            else {
                std::cout << "Fail!" << std::endl;
                return false;
            }
            calculateParameters();
            // return false;
        }
        if (!checkPathLength(q_diff)) {
            std::cout << "Error: Path length is too small" << std::endl;
            // if (DEBUG) std::cout << "q_diff: " << q_diff << "\t min_q_diff: " << min_q_diff << std::endl;
            return false;
        }
        return true;
    }

    double getVal(double x)
    {
        if (x >= t0 && x < t1)
            return diraction*ddq_m*(x - t0);
        if (x >= t1 && x < t2)
            return diraction*dq_m;
        if (x >= t2 && x < t3)
            return diraction*(dq_m - ddq_m*(x - t2));
        return 0;
    }

    void getTimeSpan(double * timeSpan)
    {
        timeSpan[0] = 0;
        timeSpan[1] = t3 + 5*min_q_diff;
    }

    bool valid;

private:
    bool calculateParameters()
    {
        // Forward time calculation
        t1 = dq_m/ddq_m + t0;
        t2 = q_diff/dq_m + t0;
        t3 = t2 + t1 - t0;

        // if (DEBUG) std::cout << "t \t [t0, t1, t2, t3] \t\t (" << t0 << ", " << t1 << ", " << t2 << ", " << t3 << ")" << std::endl;
        // if (DEBUG) std::cout << "Params \t [vel, accel, q_i, q_e] \t (" << dq_m << ", " << ddq_m << ", " << q_i << ", " << q_e << ")" << std::endl;
        return true;
    }

    bool checkTime()
    {
        if (t1 > t2) return false;
        else return true;
    }

    bool checkPathLength(double pathLength)
    {
        if (pathLength <= min_q_diff) return false;
        else  return true;
    }

    bool maxVelCorrection(double q_diff)
    {
        dq_m = 0.9*sqrt(ddq_m*q_diff);
        if (dq_m > MAX_VEL) dq_m = MAX_VEL;
        return true;
    }

    double t0, t1, t2, t3;
    double ddq_m, dq_m, q_i, q_e, min_q_diff, q_diff, diraction, shiftTime;
};

class Trajectory {
public:
    Trajectory();
    ~Trajectory();

    //fills velTra, posTra, time
    void calculateWorkSpaceTrajectory(const double maxVel, const double maxAccel,
        const Pose & startPose, const Pose & endPose, const double timeStep);
    void convertWorkSpaceToJointSpace(Pose startPose, Pose endPose, const double timeStep);
    void generateTrajectoryMsg(trajectory_msgs::JointTrajectory & trajectory);
    void exponencialMovingAvarage(std::vector<JointValues> & data, std::vector<JointValues> & movingAvarage, const double alpha);
    double getTrajectoryTime();
    void generatePowers(const int n, double* powers);
    void quinticSpline(const Vector3d & start_pos, const Vector3d & end_pos, const std::vector<double> & tr_time, const Vector3d & start_vel, const Vector3d & end_vel);
    //multisegment trajectory generation
    void mstraj(double maxVel, double dt, double maxAccel, const Pose & startPose, const std::vector<Pose> & segmentsPose);
    // Trjectory generation ONLY for point to point movement in configuration space of a robot
    bool ConfSpaceTrj(JointValues startAng, JointValues endAng, JointValues accel, JointValues vel, double T);

    std::vector<JointValues> qTra, qdotTra, qdotdotTra;
    std::vector<Vector3d> posTra, velTra, accTra;

    std::vector<double> time;
private:
    int generateTrajectory(const Pose startPose, const Pose endPose, const double timeStep, JointValues initialAngles);
    double getVel(double time);
    double t0, t1, t2, t3, maxVel, maxAccel;
    int directionSign;
    // Vector3d rotVel;
    ArmKinematics solver;
    std::vector<VelFunc> funcVector;
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator(const double maxVel, const double maxAccel, const double dt);
    ~TrajectoryGenerator();

    std::vector<JointValues> calculateTrajectory(const Pose & startPose, const Pose & endPose);
    std::vector<JointValues> calculateTrajectory(const Pose & startPose, const std::vector<Pose> & segmentsPose);

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