#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char const *argv[])
{
    Vector3d curPos, curVel, curAcc, curRot;
    JointValues angles;
    Pose startPose, endPose;
    std::vector<JointValues> sol;
    ArmKinematics solver;
    double alpha;

    // Main trajectory params
    double maxAcc = 0.05, maxVel = 0.05, timeStep = 0.01;


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

    std::ofstream logFile;
    std::string logDirPath = "/home/senserlex/test_ws/src/red_manipulation_step/trajectory_generator/";
    std::stringstream filename;
    filename << logDirPath << "logs/WorkSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());

    for (uint i = 0; i < traj.posTra.size(); ++i) {
        curPos = traj.posTra[i];
        curVel = traj.velTra[i];
        curAcc = traj.accTra[i];
        curRot = traj.rotTra[i];

        logFile << curPos(0) << "\t" << curPos(1) << "\t" << curPos(2) 
        << "\t" << curVel(0) << "\t" << curVel(1) << "\t" << curVel(2)
        << "\t" << curAcc(0) << "\t" << curAcc(1) << "\t" << curAcc(2) 
        << "\t" << curRot(0) << "\t" << curRot(1) << "\t" << curRot(2) 
        << "\t" << traj.time[i] << "\n";
    }

    return 0;
}