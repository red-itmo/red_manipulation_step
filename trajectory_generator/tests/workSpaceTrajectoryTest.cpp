#include <trajectory_generator/TrajectoryGenerator.h>
#include <ros/package.h>
#include <fstream>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "work_traj_test");
    ros::NodeHandle nh;

    std::string path;

    // Read parametars
    double maxAcc = 0.05, maxVel = 0.01, timeStep = 0.01;
    std::vector<double> initPosition, endPosition, orientation;
    bool reading = nh.getParam("/work_traj_test/start_position", initPosition);
    nh.getParam("/work_traj_test/end_position", endPosition);
    nh.getParam("/work_traj_test/angles", orientation);
    nh.getParam("/work_traj_test/a_m", maxAcc);
    nh.getParam("/work_traj_test/v_m", maxVel);
    nh.getParam("/work_traj_test/time_step", timeStep);
    path = ros::package::getPath("trajectory_generator");

    if (!reading) {
        ROS_FATAL_STREAM("[WST test] Parameters launch file is not found.");
        return 1;
    } else {
        ROS_INFO_STREAM("[WST test] Initial position ("
            << initPosition[0]
            << ", " << initPosition[1]
            << ", " << initPosition[2] << ")");
        ROS_INFO_STREAM("[WST test] End position ("
            << endPosition[0]
            << ", " << endPosition[1]
            << ", " << endPosition[2] << ")");
        ROS_INFO_STREAM("[WST test] Orientation ("
            << orientation[0]
            << ", " << orientation[1] << ")");
        ROS_INFO_STREAM("[WST test] Max vel.: " << maxVel);
        ROS_INFO_STREAM("[WST test] Max accel.: " << maxAcc);
        ROS_INFO_STREAM("[WST test] Time step: " << timeStep);
        ROS_INFO_STREAM("[WST test] Path: " << path);
    }

    // Converation variables
    Vector3d curPos, curRot;
    JointValues angles;
    Pose startPose, endPose;
    ArmKinematics solver;
    double alpha = orientation[1];


    startPose.position(0) = initPosition[0];
    startPose.position(1) = initPosition[1];
    startPose.position(2) = initPosition[2];
    startPose.orientation(0) = orientation[0];
    startPose.orientation(1) = 0;

    endPose.position(0) = endPosition[0];
    endPose.position(1) = endPosition[1];
    endPose.position(2) = endPosition[2];
    endPose.orientation(0) = orientation[1];
    endPose.orientation(1) = 0;

    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);

    std::ofstream logFile;
    std::string logDirPath = path;
    std::stringstream filename;
    filename << logDirPath << "/logs/WorkSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());

    if (!logFile.is_open())
        ROS_WARN_STREAM("file "<<  filename.str()<<" is not opened");

    ROS_INFO_STREAM("Size: " << traj.posTra.size());
    for (uint i = 0; i < traj.posTra.size(); ++i) {
        curPos = traj.posTra[i];

        logFile << curPos(0) << "\t" << curPos(1) << "\t" << curPos(2)
        << "\t" << traj.time[i] << "\n";
    }

    return 0;
}