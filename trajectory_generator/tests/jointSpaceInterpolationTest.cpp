#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char *argv[])
{
    ROS_INFO("traj test started");
    Trajectory traj;

    double a[] = {0, 0, 0, 0, 0}; //start pose
    JointValues startPose(a);

    double b[] = {1, 1, 1, 1, 1}; //via point 1
    JointValues endPose(b);

    JointValues maxAcc, maxVel;
    maxAcc.setAll(0.5);
    maxVel.setAll(0.1);

    double dt = 0.05;

    traj.ConfSpaceTrj( startPose,  endPose,  maxAcc,  maxVel,  dt);

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

    JointValues curPos, curAngVel;
    int col_width = 10;
    for (uint i = 0; i < traj.qTra.size(); ++i) {
        curPos = traj.qTra[i];
        curAngVel = traj.qdotTra[i];

        logFile << std::fixed << std::setprecision(5)
                << std::setw(col_width) << curPos(0) << std::setw(col_width) << curPos(1) << std::setw(col_width) << curPos(2)
                << std::setw(col_width) << curPos(3) << std::setw(col_width) << curPos(4)
                << std::setw(col_width) << curAngVel(0) << std::setw(col_width) << curAngVel(1) << std::setw(col_width) << curAngVel(2)
                << std::setw(col_width) << curAngVel(3) << std::setw(col_width) << curAngVel(4)
                << std::setw(col_width) << traj.time[i] << "\n";
    }
//----------end workspace logging section----------
}