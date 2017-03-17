#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char const *argv[])
{
    JointValues curJntAng, curAngVel, curAngAcc, angles, curEMA, angleWithoutOffsets;
    Pose startPose, endPose;
    std::vector<JointValues> sol;
    ArmKinematics solver;
    Vector3d zeros, smoothPos;
    double alpha = 3.1415;

    // Main trajectory params
    double maxAcc = 0.05, maxVel = 0.05, timeStep = 0.01;

    startPose.position(0) = 0.4;
    startPose.position(2) = -0.1;
    startPose.orientation(2) = alpha;
    if (!solver.solveFullyIK(startPose, angles)) {
        ROS_ERROR_STREAM("Solution start pose not found!");
        return 1;
    }

    endPose = startPose;
    endPose.position(2) = -0.15;
    if (!solver.solveFullyIK(endPose, angles)) {
        ROS_ERROR_STREAM("Solution end pose not found!");
        return 1;
    }


    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);
    traj.convertWorkSpaceToJointSpace(startPose, endPose, timeStep);

    std::ofstream logFile;
    std::string logDirPath = "/home/senserlex/youbot_ws/src/red_manipulation_step/trajectory_generator/";
    std::stringstream filename;
    filename << logDirPath << "logs/JointSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());

    for (uint i = 0; i < traj.qTra.size(); ++i) {
        curJntAng = traj.qTra[i];
        curAngVel = traj.qdotTra[i];
        curAngAcc = traj.qdotdotTra[i];
        // curEMA = traj.ema[i];
        angleWithoutOffsets = curJntAng;
        makeKinematicModelOffsets(angleWithoutOffsets);
        smoothPos = solver.transformFromFrame5ToFrame0(angleWithoutOffsets, zeros);
        logFile << curJntAng(0) << "\t" << curJntAng(1) << "\t" << curJntAng(2) << "\t" << curJntAng(3) << "\t" << curJntAng(4)
        << "\t"  << curAngVel(0) << "\t" << curAngVel(1) << "\t" << curAngVel(2) << "\t" << curAngVel(3) << "\t" << curAngVel(4)
        << "\t" << curAngAcc(0) << "\t" << curAngAcc(1) << "\t" << curAngAcc(2) << "\t" << curAngAcc(3) << "\t" << curAngAcc(4)
        << "\t" << smoothPos(0) << "\t" << smoothPos(1) << "\t" << smoothPos(2)
        << "\t" << traj.time[i] << "\n";
    }

    return 0;
}