#include <arm_manipulation/youbot_manipulator.h>
#include <trajectory_generator/TrajectoryGenerator.h>
#include <iostream>
// #include <fstream>
// #include <assert.h>
#include <string>
#include <arm_kinematics/DataTypes.h>

matrix::Matrix<double,4,4> Rot(double phi, double speed)
{
    double dz=0;//object's altitude change per time
    double a[4][4]={
        {cos(phi),-sin(phi),0,speed*cos(phi)},
        {sin(phi),cos(phi),0,speed*sin(phi)},
        {0,0,1,dz},
        {0,0,0,1}
    };
    matrix::Matrix<double,4,4> dT(a);
    return dT;
}

int main(int argc, char  ** argv)
{
    ros::init(argc, argv, "moving_belt");
    ros::NodeHandle nh;
    ROS_INFO("Starting moving_belt...");

    //[y,x,z]
    double a[4]={0.1,0.35,0.04,1.0};
    matrix::Vector<double,4> curr_coord(a);//object's start point
    double dphi=0,
    speed=-0.04,//object's speed
    t_end=3;//prediction time,s
    matrix::Vector<double,4> next_coord;
    double phi_sum=-M_PI/2;
    for(double i=0;i<t_end;i++)
    {
        std::cout<<i<<std::endl;
        next_coord=Rot(dphi,speed)*curr_coord;
        curr_coord=next_coord;
        phi_sum+=dphi;
        curr_coord.print();
    }
    phi_sum=remainder(phi_sum,M_PI);
    double maxVel=0.3, maxAcc=0.3, timeStep=0.005;
    YoubotManipulator youbotManipulator(nh);
    youbotManipulator.setConstraints(maxAcc, maxVel, timeStep);

    Pose startPose, endPose;
    //manipulator's start point
    startPose.position(0) = 0.32;
    startPose.position(1) = 0.1;
    startPose.position(2) = 0.1;
    startPose.orientation(0) = M_PI;
    startPose.orientation(1) = 0;

    //goal point to grasp object
    endPose.position(0) = curr_coord(1);
    endPose.position(1) = curr_coord(0);
    endPose.position(2) = curr_coord(2);
    endPose.orientation(0) = M_PI;
    endPose.orientation(1) = 0;

    std::cout<<"start_point:"<<std::endl;
    startPose.position.print();
    startPose.orientation.print();
    std::cout<<"end_point:"<<std::endl;
    endPose.position.print();
    endPose.orientation.print();

    Trajectory traj;
    // traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);
    std::vector<Pose> segmentsPose;
    segmentsPose.push_back(endPose);
    traj.mstraj(maxVel, timeStep, maxAcc, startPose, segmentsPose);
    std::cout<<"actual time to complete trajectory:"<< traj.getTrajectoryTime()<<std::endl;

    double timeToGrasp=0.1;
    double deltaTime=t_end-traj.getTrajectoryTime()-timeToGrasp;

    if(deltaTime<timeToGrasp){
        ROS_WARN("Not feasible to grasp!");
        deltaTime=0;
    }

    // Acception step
    std::string acception = "y";
    // std::cout << "Start trajectory test? (y, n)"; std::cin >> acception;
    // if (acception != "y") return 1;
    ros::Time startTime=ros::Time::now();

    double gropened = 0.0115;
    double grclosed = 0;
    youbotManipulator.moveGripper(gropened);

    //run conveyor simulaniously with manipulator
    // std::system("sshpass -p maker ssh -o StrictHostKeyChecking=no  robot@192.168.0.53 'python3 /home/robot/matsuev/line.py ' &");

    //time between executing ssh command and running ev3 is 9 sec
    //waiting until lego ev3 will run
    // ros::Duration(8).sleep();

    // youbotManipulator.moveToLineTrajectory(startPose, endPose);
    // std::cout<<(startTime-ros::Time::now()).toSec()<<"\n";
    // ros::Duration(deltaTime).sleep();

    //closing gripper and taking object
    // maxVel-=0.03;
    youbotManipulator.setConstraints(maxAcc, maxVel, timeStep);
    endPose.position(1)-=0.05;
    segmentsPose.push_back(endPose);
    segmentsPose.push_back(startPose);
    youbotManipulator.moveToLineTrajectory(startPose, segmentsPose);
    // Pose newendPose=endPose;
    // youbotManipulator.moveGripper(grclosed);
    // newendPose.position(1)-=0.05;
    // youbotManipulator.moveToLineTrajectory(endPose, endPose);

    //return to initial position
    // youbotManipulator.moveToLineTrajectory(endPose, startPose);

}