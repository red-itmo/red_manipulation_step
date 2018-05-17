#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

void jointSpaceTrj(Trajectory & traj, Pose startPose, double timeStep)
{
    Pose endPose;
    traj.convertWorkSpaceToJointSpace(startPose, endPose, timeStep);

    //--------jointspace logging section-----------//
    std::ofstream logFile;
    std::string path = "/home/egor/vrepWS/src/red_manipulation_step/trajectory_generator/";
    std::string logDirPath = path;
    std::stringstream filename;
    filename << logDirPath << "logs/JointSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());
    if (!logFile.is_open())
        ROS_WARN_STREAM("file " << filename.str() << " is not opened");

    JointValues curPos, curVel, curAcc;
    int col_width = 10;
    for (uint i = 0; i < traj.qTra.size(); ++i) {
        curPos = traj.qTra[i];
        curVel = traj.qdotTra[i];
        // curAngAcc = traj.qdotdotTra[i];
        logFile << std::fixed << std::setprecision(5)
                << std::setw(col_width) << curPos(0) << std::setw(col_width) << curPos(1) << std::setw(col_width) << curPos(2)
                << std::setw(col_width) << curPos(3) << std::setw(col_width) << curPos(4)
                << std::setw(col_width) << curVel(0) << std::setw(col_width) << curVel(1) << std::setw(col_width) << curVel(2)
                << std::setw(col_width) << curVel(4) << std::setw(col_width) << curVel(4)
                << std::setw(col_width) << traj.time[i] << "\n";
    }
//----------end jointspace logging section-----------
}

int main(int argc, char *argv[])
{
    ROS_INFO("traj test started");
    Trajectory traj;

    double a[] = {0.32, 0.1, 0.1}; //start pose
    Vector3d aV(a);
    Pose q0;
    q0.position = aV;
    q0.orientation(0) = 3.14;

    double b[] = {0.35, -0.02, 0.04}; //via point 1
    Pose q1;
    q1.position = b;

    double c[] = {0.25, -0.02, 0.04}; //via point 2
    Pose q2;
    q2.position = c;

    double d[] = {0.35, -0.02, 0.04}; //via point 3
    Pose q3;
    q3.position = d;

    double maxAcc = 0.2, qdMax = 0.2;
    std::vector<Pose> segmentsPose;
    segmentsPose.push_back(q1);
    segmentsPose.push_back(q2);
    segmentsPose.push_back(q3);


    double dt = 0.05;
    traj.mstraj(qdMax, dt, maxAcc, q0, segmentsPose);

//--------workspace logging section-----------//
    std::ofstream logFile;
    std::string path = "/home/egor/vrepWS/src/red_manipulation_step/trajectory_generator/";
    std::string logDirPath = path;
    std::stringstream filename;
    filename << logDirPath << "logs/WorkSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());
    if (!logFile.is_open())
        ROS_WARN_STREAM("file " << filename.str() << " is not opened");

    Vector3d curPos, curVel, curAcc;
    int col_width = 10;
    for (uint i = 0; i < traj.posTra.size(); ++i) {
        curPos = traj.posTra[i];
        curVel = traj.velTra[i];
        curAcc = traj.accTra[i];
        // curAngAcc = traj.qdotdotTra[i];
        logFile << std::fixed << std::setprecision(5)
                << std::setw(col_width) << curPos(0) << std::setw(col_width) << curPos(1) << std::setw(col_width) << curPos(2)
                << std::setw(col_width) << curVel(0) << std::setw(col_width) << curVel(1) << std::setw(col_width) << curVel(2)
                << std::setw(col_width) << curAcc(0) << std::setw(col_width) << curAcc(1) << std::setw(col_width) << curAcc(2)
                << std::setw(col_width) << traj.time[i] << "\n";
    }
//----------end workspace logging section-----------
jointSpaceTrj(traj, q0, dt);
}