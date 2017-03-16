
#include <arm_manipulation/youbot_manipulator.h>
#include <trajectory_generator/TrajectoryGenerator.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <assert.h>

ArmKinematics solver;
double epsilon = 0.01;

// void logToFile(TrajectoryGenerator & gen, size_t i)
// {
//     std::ofstream logFile;      
//     std::string logDirPath = "/home/senserlex/test_ws/src/red_manipulation_step/arm_manipulation/logs/";
//     std::stringstream filename;
//     filename << logDirPath << "data" << i << ".log";
//     std::string file = filename.str();
//     logFile.open(file.c_str());

//     std::vector<Vector3d> posTraj = gen.posTra;
//     std::vector<JointValues> qTraj = gen.qTra;
//     std::vector<JointValues> qdotTraj = gen.qdotTra;
//     std::vector<JointValues> qdotdotTraj = gen.qdotdotTra;
//     double dt = gen.timeStep; 
//     std::cout << dt << std::endl;   

//     std::vector<Vector3d>::iterator posTraIT = posTraj.begin();
//     std::vector<JointValues>::iterator qTraIT = qTraj.begin();
//     std::vector<JointValues>::iterator qdotTraIT = qdotTraj.begin();
//     std::vector<JointValues>::iterator qdotdotTraIT = qdotdotTraj.begin();

//     Vector3d pos;
//     Vector3d poseFK;
//     Vector3d poseDiff;
//     JointValues jointAngles;
//     JointValues jointAnglVel;
//     JointValues jointAnglAccel;
//     Vector3d zeroPos;

//     for (posTraIT, qTraIT; posTraIT != posTraj.end(), qTraIT != qTraj.end(); ++posTraIT, ++qTraIT) {
//         pos = *posTraIT;
//         jointAngles = *qTraIT;
//         jointAnglVel = *qdotTraIT;
//         jointAnglAccel = *qdotdotTraIT;

//         makeKinematicModelOffsets(jointAngles);

//         poseFK = solver.transformFromFrame5ToFrame0(jointAngles, zeroPos);

//         logFile << "\t" << jointAnglAccel(0) 
//         << "\t" << jointAnglAccel(1) 
//         << "\t" << jointAnglAccel(2) 
//         << "\t" << jointAnglAccel(3) 
//         << "\t" << jointAnglAccel(4) 
//         << "\t" << jointAnglVel(0) 
//         << "\t" << jointAnglVel(1) 
//         << "\t" << jointAnglVel(2) 
//         << "\t" << jointAnglVel(3) 
//         << "\t" << jointAnglVel(4)
//         << "\t" << jointAngles(0) 
//         << "\t" << jointAngles(1) 
//         << "\t" << jointAngles(2) 
//         << "\t" << jointAngles(3) 
//         << "\t" << jointAngles(4) << "\n";

//         poseDiff = poseFK - pos;
//         assert(poseDiff.norm() < epsilon);

//         ++qdotTraIT;
//         ++qdotdotTraIT;
//     }
// }

int main(int argc, char  ** argv)
{
    ros::init(argc, argv, "trajectory_test");
    ros::NodeHandle nh;

    double lr = 100;

    double maxVel = 0.05;
    double maxAccel = 0.1;
    double startVel = 0;
    double endVel = 0;

    double bounds[2][2];
    bounds[0][0] = 0.3;
    bounds[0][1] = -0.1;
    bounds[1][0] = 0.32;
    bounds[1][1] = 0.1;

    double wall = -0.15;

    Pose point;
    std::vector<Pose> poseArray;

    ROS_INFO_STREAM("Generate grid points.");
    for (double x = bounds[0][0]; x <= bounds[1][0]; x += 0.01) {
        for (double y = bounds[0][1]; y <= bounds[1][1]; y += 0.01) {
            point.position(0) = x;
            point.position(1) = y;
            point.position(2) = wall + 0.05;
            point.orientation(2) = 3.1415;
            poseArray.push_back(point);
            std::cout << "p( " << x << ", " << y << ", " << (wall + 0.03) << ")" << std::endl;
        }
    }

    YoubotManipulator manipulator(nh);
    std::vector<JointValues> sol;
    JointValues angles;
    TrajectoryGenerator gen(maxVel, maxAccel, 1/lr);

    double alpha = 0;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/velocity_joint_trajecotry", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO_STREAM("GO to INITIAL position");
    // GO to initial pose
    std::vector<Pose>::iterator it = poseArray.begin();
    Pose startPos = *it;
    if (!solver.solveIK(startPos, sol)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPos.position(0) << ", " << startPos.position(1)  << ", " << startPos.position(2));
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    startPos.orientation(2) = alpha;
    manipulator.moveArm(startPos);
    ros::Duration(2).sleep();

    ROS_INFO_STREAM("Start main loop.");
    // Main loop
    size_t i = 0;
    for (it; it != poseArray.end(); ++it) {

        control_msgs::FollowJointTrajectoryGoal msg;
        trajectory_msgs::JointTrajectory lineTrajectory;

        startPos = *it;
        if (!solver.solveIK(startPos, sol)) {
            ROS_FATAL_STREAM("Solution is not found (startPos): " << startPos.position(0) << ", " << startPos.position(1)  << ", " << startPos.position(2));
            return 1;
        }
        angles = sol[0];
        alpha = angles(1) + angles(2) + angles(3);
        startPos.orientation(2) = alpha;
        manipulator.moveArm(startPos);
        ros::Duration(1).sleep();

        Pose endPos = startPos;
        endPos.position(2) = wall;
        if (!solver.solveIK(endPos, sol)) {
            ROS_FATAL_STREAM("Solution is not found (endPos): " << endPos.position(0) << ", " << endPos.position(1)  << ", " << endPos.position(2));
            return 1;
        }
        angles = sol[0];
        alpha = angles(1) + angles(2) + angles(3);
        endPos.orientation(2) = alpha;

        std::cout << "start p( " << startPos.position(0) << ", " << startPos.position(1) << ", " << startPos.position(2) << ")" << std::endl;
        std::cout << "end p( " << endPos.position(0) << ", " << endPos.position(1) << ", " << endPos.position(2) << ")" << std::endl;

        gen.calculateTrajectory(startPos, endPos);

        msg.trajectory = gen.trajectory;
        msg.trajectory.header.frame_id = "arm_link_0";
        msg.trajectory.header.stamp = ros::Time::now();
        ROS_INFO("Start trajectory execute!");
        ac.sendGoal(msg);
        ac.waitForResult(ros::Duration(10));
        manipulator.moveArm(startPos);
        ros::Duration(1).sleep();

        // ROS_INFO_STREAM("Write to file: " << "data" << i << ".log");
        // logToFile(traj, i);
        ++i;
    }

    return 0;
}