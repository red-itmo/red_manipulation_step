#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char const *argv[])
{
    JointValues curJntAng, curAngVel, curAngAcc, angles, curEMA, angleWithoutOffsets;
    Pose startPose, endPose;
    std::vector<JointValues> sol;
    ArmKinematics solver;
    Vector3d zeros, smoothPos;
    double alpha = 0;

    // Main trajectory params
    double maxAcc = 0.5, maxVel = 0.05, timeStep = 0.01;

    startPose.position(0) = 0.4;
    startPose.position(2) = -0.1;
    startPose.orientation(2) = 3.1415;
    if (!solver.solveIK(startPose, sol)) {
        ROS_ERROR_STREAM("Solution start pose not found!");
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    startPose.orientation(2) = alpha;

    endPose = startPose;
    endPose.position(2) = -0.15;
    endPose.orientation(2) = alpha;
    if (!solver.solveIK(startPose, sol)) {
        ROS_ERROR_STREAM("Solution end pose not found!");
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    endPose.orientation(2) = alpha;


    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);
    traj.convertWorkSpaceToJointSpace(timeStep);


    std::ofstream logFile;
    std::string logDirPath = "/home/senserlex/test_ws/src/red_manipulation_step/trajectory_generator/";
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