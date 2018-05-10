#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char *argv[])
{
    ROS_INFO("traj test started");
    Trajectory traj;
    double a[] = {0.35, 0.1, -0.05}; //start pose
    Vector3d aV(a);
    Pose q0;
    q0.position = aV;

    double b[] = {0.2, -0.1, 0.25}; //via point 1
    Pose q1;
    q1.position = b;
    double c[] = {0.35, 0.1, -0.05}; //via point 2
    Pose q2;
    q2.position = c;
    double d[] = {0.25, 0.2, 0.1}; //via point 3
    Pose q3;
    q3.position = d;

    double maxAcc = 0.1, qdMax = 0.1;
    std::vector<Pose> segmentsPose;
    segmentsPose.push_back(q1);
    segmentsPose.push_back(q2);
    segmentsPose.push_back(q3);


    double dt = 0.05;
    traj.mstraj(qdMax, dt, maxAcc, q0, segmentsPose);

//--------logging section-----------//
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
}