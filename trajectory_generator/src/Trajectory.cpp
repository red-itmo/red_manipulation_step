#include <trajectory_generator/TrajectoryGenerator.h>

Trajectory::Trajectory()
{}

Trajectory::~Trajectory()
{}

void Trajectory::exponencialMovingAvarage(std::vector<JointValues> & data, std::vector<JointValues> & movingAvarage, const double alpha)
{
    size_t n = data.size();
    size_t nAvg = movingAvarage.size();
    if (n != nAvg)
        movingAvarage.resize(n);

    movingAvarage[0] = data[0];
    for (size_t i = 1; i < n; ++i) {
        movingAvarage[i] = alpha * data[i] + (1 - alpha) * movingAvarage[i - 1];
    }
}

double Trajectory::getVel(double time)
{
    if (time >= 0 && time < t1)
        return directionSign * maxAccel * time;
    if (time >= t1 && time < t2)
        return directionSign * maxVel;
    if (time >= t2 && time <= t3)
        return directionSign * (maxVel - maxAccel * (time - t2));
    return 0;
}

void Trajectory::calculateWorkSpaceTrajectory(const double maxVel, const double maxAccel, const Pose & startPose, const Pose & endPose, const double timeStep)
{
    this->maxAccel = maxAccel;
    this->maxVel = maxVel;
    Vector3d positionDiff = endPose.position - startPose.position;
    Vector3d movementDirection = positionDiff;
    movementDirection.normalize();
    directionSign = std::abs(positionDiff.norm()) / positionDiff.norm();

    ROS_DEBUG_STREAM("[trajectory] Max Vel: " << maxVel << " Max Accel: " << maxAccel << " TimeStep: " << timeStep);
    // Calculate trajectory times
    t1 = maxVel / maxAccel;
    t2 = positionDiff.norm() / maxVel;
    //if no difference between start and end positions
    if(t2==0){
        ROS_WARN_STREAM("Start and end pose coincide!");
        t2=t1+0.01;
    }
    t3 = t2 + t1;


    ROS_INFO_STREAM("[Trajectory] Time variables (" << t1 << ", " << t2 << ", " << t3 << ")");

    if (t2 < t1)
    {
        ROS_FATAL_STREAM("[Trajectory]t2<t1. Try to decrease max speed");
        return;
    }
    Vector3d currCoord = startPose.position;
    Vector3d currVel;

    for (double currentTime = 0; currentTime < t3; currentTime += timeStep)
    {
        currVel = movementDirection * getVel(currentTime);
        velTra.push_back(currVel);
        posTra.push_back(currCoord);
        time.push_back(currentTime);
        currCoord += timeStep / 2 * (currVel + movementDirection * getVel(currentTime + timeStep));
    }
    ROS_INFO_STREAM("[trajectory]trj is generated, number of steps:"<<posTra.size());
}

void Trajectory::convertWorkSpaceToJointSpace(const Pose startPose, Pose endPose, const double timeStep)
{
    JointValues currJntAng, currJntAngVel, currJntAngAcc;
    JointValues prevAngles, currAngles, nextAngles, anglesDiff, zeroVec;
    prevAngles.setAll(-1000);
    ArmKinematics solver;
    double prevTime, currTime, nextTime;

    Vector3d rotVel, startRot, endRot, currRot, currPosition;
    double startAlpha, endAlpha, alpha, alphaVel;

    //filling poses vector
    std::vector<Pose> poses;
    Pose pose;
    for (int i = 0; i < posTra.size(); ++i) {
        pose.position = posTra[i];
        poses.push_back(pose);
        // pose.position.print();
        // pose.orientation.print();
    }

    double err = 0, offset = 0, theta;
    Pose curConf;
    Vector3d ang;
    JointValues curRot, angle;
    angle(1) = M_PI / 3;
    angle(2) = M_PI / 6;
    angle(3) = M_PI / 6;

    for (size_t i = 0; i < poses.size(); ++i)
    {
        curConf = poses[i];
        ang = solver.calcMaxRot(curConf.position);

        theta = ang(0) + ang(1) + ang(2);
        offset = 0;
        if (theta > startPose.orientation(0) - 0.1)
            offset = pow(theta + 0.1 - startPose.orientation(0), 2);
        else
            offset = 0;
        curConf.orientation(0) = theta - 0.1 - offset;
        curRot = solver.numericalIK(curConf, angle);
        //if error has occured
        if(curRot(0)==-1000 || curRot(0)==-2000){
            ROS_WARN_STREAM("Trajectory soluion not found!");
            return;
        }
        makeYoubotArmOffsets(curRot);
        //if the first iteration
        if(prevAngles(0)==-1000)
            prevAngles=curRot;
        currJntAngVel = (curRot - prevAngles) / timeStep;
        prevAngles = curRot;
        qdotTra.push_back(currJntAngVel);
        qTra.push_back(curRot);

        angle = solver.prevNumIKAngle;
    }
}

void Trajectory::generateTrajectoryMsg(trajectory_msgs::JointTrajectory & trajectory)
{
    ROS_INFO("Generating message...");
    JointValues currJntAng, currJntAngAcc, currJntAngVel;
    double timeStep = time[1] - time[0];

    for (size_t p = 0; p < qTra.size(); ++p) {

        trajectory_msgs::JointTrajectoryPoint point;

        currJntAng = qTra[p];
        currJntAngVel = qdotTra[p];
        //TODO acceleration
        //currJntAngAcc = qdotdotTra[p];

        for (size_t i = 0; i < 5; ++i) {
            point.positions.push_back(currJntAng(i));
            // ROS_DEBUG_ONCE("[trajectory]Publishing trajectory angles:");
            // ROS_DEBUG("[trajectory]%f",currJntAng(i));
            point.velocities.push_back(currJntAngVel(i));
            point.accelerations.push_back(currJntAngAcc(i));
            point.time_from_start = ros::Duration(p * timeStep);
        }
        trajectory.points.push_back(point);
    }

    if (trajectory.joint_names.size() == 5) return;
    else trajectory.joint_names.resize(5);

    for (size_t i = 0; i < 5; ++i) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        trajectory.joint_names[i] = jointName.str();
    }
}