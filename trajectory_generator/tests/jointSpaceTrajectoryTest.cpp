#include <trajectory_generator/TrajectoryGenerator.h>
#include <fstream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joint_traj_test");
    ros::NodeHandle nh;

    // Read parametars
    double maxAcc = 0.05, maxVel = 0.01, timeStep = 0.01;
    std::vector<double> initPosition, endPosition, orientation;
    bool reading = nh.getParam("/joint_traj_test/start_position", initPosition);
    nh.getParam("/joint_traj_test/end_position", endPosition);
    nh.getParam("/joint_traj_test/angles", orientation);
    nh.getParam("/joint_traj_test/a_m", maxAcc);
    nh.getParam("/joint_traj_test/v_m", maxVel);
    nh.getParam("/joint_traj_test/time_step", timeStep);

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

    JointValues curJntAng, curAngVel, curAngAcc, angles, curEMA, angleWithoutOffsets;
    JointValues nextAngle, angleDiff; // TEST
    Pose startPose, endPose;
    std::vector<JointValues> sol;
    ArmKinematics solver;
    Vector3d zeros, smoothPos;
    double alpha = orientation[1];
    double alpha1, alpha2;

    startPose.position(0) = initPosition[0];
    startPose.position(1) = initPosition[1];
    startPose.position(2) = initPosition[2];
    startPose.orientation(2) = alpha;
    if (!solver.solveFullyIK(startPose, angles)) {
        ROS_ERROR_STREAM("Solution start pose not found!");
        return 1;
    }
    alpha1 = angles(1) + angles(2) + angles(3);
    ROS_INFO_STREAM("[WST test] Start alpha: " << alpha1);

    endPose = startPose;
    endPose.position(0) = endPosition[0];
    endPose.position(1) = endPosition[1];
    endPose.position(2) = endPosition[2];
    endPose.orientation(2) = alpha1;
    if (!solver.solveFullyIK(endPose, angles)) {
        ROS_ERROR_STREAM("Solution end pose not found!");
        return 1;
    }
    alpha2 = angles(1) + angles(2) + angles(3);
    ROS_INFO_STREAM("[WST test] End alpha: " << alpha2);

    if (alpha1 != alpha2) {
        startPose.orientation(2) = alpha2;
        if (!solver.solveFullyIK(startPose, angles)) {
            ROS_ERROR_STREAM("Solution start pose not found!");
            return 1;
        }
        alpha1 = angles(1) + angles(2) + angles(3);
        ROS_INFO_STREAM("[WST test] Start alpha: " << alpha1);
    }

    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);
    traj.convertWorkSpaceToJointSpace(startPose, endPose, timeStep);

    std::ofstream logFile;
    std::string logDirPath = "/home/senex/youbot_ws/src/red_manipulation_step/trajectory_generator/";
    std::stringstream filename;
    filename << logDirPath << "logs/JointSpaceTraj" << ".log";
    std::string file = filename.str();
    logFile.open(file.c_str());

    // TEST
    // for (uint i = 0; i < traj.qTra.size(); ++i) {
    //     curJntAng = traj.qTra[i];
    //     curAngVel = traj.qdotTra[i];
    //     curAngAcc = traj.qdotdotTra[i];
    //     // curEMA = traj.ema[i];
    //     angleWithoutOffsets = curJntAng;
    //     if (i < traj.qTra.size() - 1) {
    //         nextAngle = traj.qTra[i + 1];
    //     } else nextAngle = traj.qTra[i];
    //     angleDiff = nextAngle - curJntAng;
    //     if (angleDiff(1) > 0.03) {
    //         for(int j = i; j >= 0; --j) {
    //             traj.qTra[j] += angleDiff;
    //         }
    //     }
    // }

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