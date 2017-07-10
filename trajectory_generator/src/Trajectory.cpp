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
        movingAvarage[i] = alpha*data[i] + (1 - alpha)*movingAvarage[i - 1];
    }
}

void Trajectory::calculateWorkSpaceTrajectory(const double maxVel, const double maxAccel, const Pose & startPose, const Pose & endPose, const double timeStep)
{
    Vector3d positionDiff, rotationDiff, movementDirection, rotVel, currPos, currVel, currAccel, currRot, initPos, initVel;
    const double acceleration[4] = {0, maxAccel, 0, -maxAccel};

    initPos = startPose.position;
    positionDiff = endPose.position - initPos;
    movementDirection = positionDiff;
    movementDirection.normalize();

    // Calculate trajectory times        
    const double trajectoryTime[4] = {
        0,
        maxVel/maxAccel,                                                    // acceleration = maxAccel; start velocity = 0;
        positionDiff.norm()/maxVel,                                         // acceleration = 0; velocity = maxVel;
        (maxVel*maxVel + positionDiff.norm()*maxAccel)/(maxAccel*maxVel)    // acceleration = -maxAccel; end velocity = 0;
    };
    ROS_INFO_STREAM("Time variables: " << trajectoryTime[1] << ", " << trajectoryTime[2] << ", " << trajectoryTime[3]);

    // Initial conditions
    double currentTime = 0;

    for (size_t i = 1; i < 4; ++i) {

        currAccel = movementDirection * acceleration[i];
        currVel = initVel;
        currPos = initPos;

        while ((trajectoryTime[i] - currentTime) > 0.0001) {

            currVel = initVel + currAccel * (currentTime - trajectoryTime[i - 1]);
            currPos = initPos + initVel * (currentTime - trajectoryTime[i - 1]) + currAccel * pow(currentTime - trajectoryTime[i - 1], 2) / 2;

            posTra.push_back(currPos);
            velTra.push_back(currVel);
            accTra.push_back(currAccel);
            time.push_back(currentTime);
            // ROS_INFO_STREAM("p: " << currPos(0) << ", " << currPos(1) << ", " << currPos(2) << "\ttime: " << currentTime << "\tDelta time: " << (trajectoryTime[i] - currentTime));

            currentTime += timeStep;
        }

        currentTime = trajectoryTime[i];
        if ((currentTime - trajectoryTime[i - 1]) < 0.0001) {
            currentTime += timeStep;
            continue;
        }

        initPos = initPos + initVel * (currentTime - trajectoryTime[i - 1]) + currAccel * pow(currentTime - trajectoryTime[i - 1], 2) / 2;
        initVel = initVel + currAccel * (currentTime - trajectoryTime[i - 1]);

        posTra.push_back(initPos);
        velTra.push_back(initVel);
        accTra.push_back(currAccel);
        time.push_back(currentTime);
        // ROS_INFO_STREAM("accel: " << acceleration[i] << "\ttime: " << currentTime << "\tvel: " << initVel(0) << ", " << initVel(1) << ", " << initVel(2));

        currentTime += timeStep;
    }
}
void Trajectory::convertWorkSpaceToJointSpace(const Pose & startPose, const Pose & endPose, const double timeStep)
{
    JointValues currJntAng, currJntAngVel, currJntAngAcc;
    JointValues prevAngles, currAngles, nextAngles;
    ArmKinematics solver;
    double prevTime, currTime, nextTime;
    bool err = false;

    Vector3d rotVel, startRot, endRot, currRot, currPosition;
    double startAlpha, endAlpha, alpha, alphaVel;

    startRot = startPose.orientation;
    endRot = endPose.orientation;

    for (int configuration = -1; configuration < 2; configuration += 2) {
        rotTra.clear();
        qTra.clear();

        if (solver.solveIK(startPose, currJntAng, configuration)) 
            startAlpha = currJntAng(1) + currJntAng(2) + currJntAng(3);
        else {
            ROS_WARN_STREAM("Solution for start position with configuration: " << configuration << " is not found");
            continue;
        }

        if (solver.solveIK(endPose, currJntAng, configuration)) 
            endAlpha = currJntAng(1) + currJntAng(2) + currJntAng(3);
        else {
            ROS_WARN_STREAM("Solution for end position with configuration: " << configuration << " is not found");
            continue;
        }

        startRot(2) = startAlpha;
        endRot(2) = endAlpha;
        rotVel = (endRot - startRot)/(posTra.size() - 1);

        currRot = startRot;
        for (size_t i = 0; i < posTra.size(); ++i) {
            rotTra.push_back(currRot);
            currRot += rotVel;
        }

        std::vector<Vector3d>::iterator posIT = posTra.begin();
        std::vector<Vector3d>::iterator velIT = velTra.begin();
        std::vector<Vector3d>::iterator rotIT = rotTra.begin();

        for (posIT; posIT != posTra.end(); ++posIT, ++velIT, ++rotIT) {
            if (!solver.solveIK(*posIT, *rotIT, currJntAng, configuration)) {
                ROS_WARN_STREAM("Solution is not found at configuration: " << configuration);
                err = true; 
                break;
            }
            makeYoubotArmOffsets(currJntAng);

            qTra.push_back(currJntAng);
        }

        if (err == true) continue;

        std::vector<JointValues>::iterator pp = qTra.begin() - 1;
        std::vector<JointValues>::iterator np = qTra.begin() + 1;

        for (std::vector<JointValues>::iterator cp = qTra.begin(); cp != qTra.end(); ++cp) {
            if (pp == qTra.begin() - 1) {
                prevAngles = *cp;
                // prevTime = *ct;
            } else {
                prevAngles = *pp;
                // prevTime = *pt;
            }
            currAngles = *cp;
            // currTime = *ct;
            if (np == qTra.end()) {
                nextAngles = *cp;
                // nextTime = *ct;
            } else {
                nextAngles = *np;
                // nextTime = *nt;
            }

            //forward difference for velocity
            currJntAngVel = (nextAngles - currAngles) / timeStep;

            //second derivative for acceleration (f(x+h)-2*f(x)+f(x-h))/timeStep^2
            currJntAngAcc = (nextAngles - 2.0 * currAngles + prevAngles) / pow(timeStep, 2);

            qdotTra.push_back(currJntAngVel);
            qdotdotTra.push_back(currJntAngAcc);
            ++pp; ++np;
            // ++pt; ++ct; ++nt;
        }
        
        currJntAngVel.setZero();
        qdotTra.erase(qdotTra.begin());
        qdotTra.erase(qdotTra.end());
        qdotTra.insert(qdotTra.begin(), currJntAngVel);
        qdotTra.insert(qdotTra.end(), currJntAngVel);
        currJntAngAcc.setZero();
        qdotdotTra.erase(qdotdotTra.begin());
        qdotdotTra.erase(qdotdotTra.end());
        qdotdotTra.insert(qdotdotTra.begin(), currJntAngAcc);
        qdotdotTra.insert(qdotdotTra.end(), currJntAngAcc);

        return;
    }
    ROS_FATAL_STREAM("Trajectory is not construct.");
}

void Trajectory::generateTrajectoryMsg(trajectory_msgs::JointTrajectory & trajectory)
{
    JointValues currJntAng, currJntAngAcc, currJntAngVel;
    double timeStep = time[1] - time[0];
    
    for (size_t p = 0; p < qTra.size(); ++p) {
        trajectory_msgs::JointTrajectoryPoint point;

        currJntAng = qTra[p];
        currJntAngAcc = qdotTra[p];
        currJntAngVel = qdotdotTra[p];

        for (size_t i = 0; i < 5; ++i) {
            point.positions.push_back(currJntAng(i));
            point.velocities.push_back(currJntAngAcc(i));
            point.accelerations.push_back(currJntAngVel(i));
            point.time_from_start = ros::Duration(p*timeStep);
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