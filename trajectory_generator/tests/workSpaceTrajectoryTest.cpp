#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "work_traj_test");
    ros::NodeHandle nh;

    // Read parametars
    double maxAcc = 0.05, maxVel = 0.01, timeStep = 0.01;
    std::vector<double> initPosition, endPosition, orientation;
    bool reading = nh.getParam("/work_traj_test/start_position", initPosition);
    nh.getParam("/work_traj_test/end_position", endPosition);
    nh.getParam("/work_traj_test/angles", orientation);
    nh.getParam("/work_traj_test/a_m", maxAcc);
    nh.getParam("/work_traj_test/v_m", maxVel);
    nh.getParam("/work_traj_test/time_step", timeStep);

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
    }

    // Converation variables
    Vector3d curPos;
    JointValues angles;
    Pose startPose, endPose;
    ArmKinematics solver;
    double alpha = orientation[1];


    startPose.position(0) = initPosition[0];
    startPose.position(2) = initPosition[2];
    startPose.orientation(2) = alpha;
    if (!solver.solveFullyIK(startPose, angles)) {
        ROS_ERROR_STREAM("Solution start pose not found!");
        return 1;
    }
    ROS_INFO_STREAM("[WST test] Start alpha: " << (angles(1) + angles(2) + angles(3)));

    endPose.position(0) = endPosition[0];
    endPose.position(2) = endPosition[2];
    endPose.orientation(2) = alpha;
    if (!solver.solveFullyIK(endPose, angles)) {
        ROS_ERROR_STREAM("Solution end pose not found!");
        return 1;
    }
    ROS_INFO_STREAM("[WST test] End alpha: " << (angles(1) + angles(2) + angles(3)));

    ROS_INFO_STREAM("[WST test] Start position (" << startPose.position(0) << ", " 
        << startPose.position(1) << ", " <<  startPose  .position(2) << ")");
    ROS_INFO_STREAM("[WST test] End position (" << startPose.position(0) << ", " 
        << endPose.position(1) << ", " <<  endPose.position(2) << ")");
    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);

    std::ofstream logFile;
    std::string logDirPath = "/home/senex/youbot_ws/src/red_manipulation_step/trajectory_generator/";
    std::stringstream filename;
    filename << logDirPath << "logs/WorkSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());

    ROS_INFO_STREAM("Size: " << traj.posTra.size());
    for (uint i = 0; i < traj.posTra.size(); ++i) {
        curPos = traj.posTra[i];

        logFile << curPos(0) << "\t" << curPos(1) << "\t" << curPos(2) 
        << "\t" << traj.time[i] << "\n";
    }

    return 0;
}