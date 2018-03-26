#include <arm_manipulation/youbot_manipulator.h>
#include <trajectory_generator/TrajectoryGenerator.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
// Simple action client at the arm_manipulation/youbot_manipulator.h

int main(int argc, char  ** argv)
{
    ros::init(argc, argv, "trajectory_test");
    ros::NodeHandle nh;
    

    // Trajectory params
    double timeStep = 0.05, maxVel = 0.05, maxAcc = 0.1;

    nh.param("/trajectory_test/a_m", maxAcc, 0.1);
    nh.param("/trajectory_test/v_m", maxVel, 0.05);
    nh.param("/trajectory_test/time_step", timeStep, 0.2);    

    std::vector<Pose> startPoses;
    double xMin = 0.35, xMax = 0.35;
    double yMin = -0.2, yMax = 0.1;
    double step = 0.05; 
    Pose p1,p2,p;
    double alpha = 3.1415;

    ROS_INFO_STREAM("[TRJ test] Generate points grid");

    // for (double x = xMin; x <= xMax; x += step) {
    //     for (double y = yMin; y <= yMax; y += step) {
    //         ROS_INFO_STREAM("(x, y) -- (" << x << ", " << y << ")");
    //         p.position(0) = x;
    //         p.position(1) = y;
    //         p.position(2) = 0;
    //         p.orientation(2) = alpha;

    //         startPoses.push_back(p);
    //     }
    // }

    double x0 = 0.25, y0 = 0.25, z0 = 0.0, radius=0.1;
    for (double angle = 0; angle <= 360; angle += 5) {
        p.position(0) = x0+radius*cos(angle*3.14/180);
        p.position(1) = y0+radius*sin(angle*3.14/180);
        p.position(2) = z0;
        p.orientation(2) = alpha;
        ROS_INFO_STREAM("(x, y, z) -- (" << p.position(0) << ", " << p.position(1) << ", " << p.position(2) << ")");
        startPoses.push_back(p);
    }

    // p1.position(0) = 0.56;
    // p1.position(1) = -0.12;
    // p1.position(2) = 0.135;
    // p1.orientation(1) = 0.1;
    // p1.orientation(2) = 1.6;
    

    // p2.position(0) = -0.3;
    // p2.position(1) = 0;
    // p2.position(2) = 0.3;
    // p2.orientation(2) = -2;

    // startPoses.push_back(p1);
    // startPoses.push_back(p2);
    // startPoses.push_back(p1);

    ArmKinematics solver;
    YoubotManipulator manipulator(nh);
    manipulator.initArmTopics();
    manipulator.initActionClient(maxAcc, maxVel, timeStep);

    // Acception step
    std::string acception="y";
    std::cout << "Start trajectory test? (y, n)"; std::cin >> acception;
    if (acception != "y") return 1;

    // Execution step
    Pose startPose, endPose;
    JointValues angles;
    int i = 0;
    double alpha1, alpha2;

    while (nh.ok() && (i < startPoses.size()-1)) {

        ROS_INFO_STREAM("[TRJ test] Point (" << startPoses[i].position(0) 
            << ", " << startPoses[i].position(1) 
            << ", " << startPoses[i].position(2) << ")");

        startPose = startPoses[i];
        if (!solver.solveFullyIK(startPose, angles)) {
            ROS_ERROR_STREAM("Solution start pose not found!");
            return 1;
        }
        alpha1 = angles(1) + angles(2) + angles(3);

        endPose = startPoses[i+1];
        // endPose.position(2) = -0.05;
        // endPose.orientation(2) = alpha1;
        //endPose.orientation(2) = alpha;
        if (!solver.solveFullyIK(endPose, angles)) {
            ROS_ERROR_STREAM("Solution end pose not found!");
            return 1;
        }
        alpha2 = angles(1) + angles(2) + angles(3);

        // if (alpha1 != alpha2) {
        //     startPose.orientation(2) = alpha2;
        //     if (!solver.solveFullyIK(startPose, angles)) {
        //         ROS_ERROR_STREAM("Solution start pose not found!");
        //         return 1;
        //     }
        //     alpha1 = angles(1) + angles(2) + angles(3);
        //     ROS_INFO_STREAM("[WST test] Start alpha: " << alpha1);
        // }   

        manipulator.moveArm(startPose);
        // manipulator.moveGripper(0.0115);
         
        // ros::Duration(2).sleep();
         
        manipulator.moveToLineTrajectory(startPose, endPose);

        manipulator.moveArm(endPose);
         
        //ros::Duration(0.5).sleep();
         

        ++i;
    }
    ROS_INFO_STREAM("END");
    //killing program
    // system("rosnode kill -a"); 
    // system("killall roslaunch"); 

    return 0;
}