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

    double a[4]={0.26,0,0.1,1.0};
    matrix::Vector<double,4> curr_coord(a);//object's start point
    double dphi=0,
    speed=0.01,//object's speed
    t_end=3;//prediction time,s
    matrix::Vector<double,4> next_coord;
    double phi_sum;
    for(double i=0;i<t_end;i++)
    {
        std::cout<<i<<std::endl;
        next_coord=Rot(dphi,speed)*curr_coord;
        curr_coord=next_coord;
        phi_sum+=dphi;
        curr_coord.print();
    }
    phi_sum=remainder(phi_sum,2*M_PI);
    double maxVel=0.05, maxAcc=0.1, timeStep=0.2;
    YoubotManipulator youbotManipulator(nh);
    youbotManipulator.setConstraints(maxAcc, maxVel, timeStep);

    Pose startPose, endPose;
    //manipulator's start point
    startPose.position(0) = 0.26;
    startPose.position(1) = 0;
    startPose.position(2) = 0.2;
    startPose.orientation(0) = M_PI;
    startPose.orientation(1) = 0;

    //goal point to grasp object
    endPose.position(0) = curr_coord(0);
    endPose.position(1) = curr_coord(1);
    endPose.position(2) = curr_coord(2);
    endPose.orientation(0) = M_PI;
    endPose.orientation(1) = phi_sum;

    std::cout<<"start_point:"<<std::endl;
    startPose.position.print();
    startPose.orientation.print();
    std::cout<<"end_point:"<<std::endl;
    endPose.position.print();
    endPose.orientation.print();

    Trajectory traj;
    traj.calculateWorkSpaceTrajectory(maxVel, maxAcc, startPose, endPose, timeStep);
    std::cout<<"actual time to complete trajectory:"<< traj.getTrajectoryTime()<<std::endl;

    double deltaTime=t_end-traj.getTrajectoryTime();

    if(deltaTime<0.1){
        ROS_WARN("Not feasible to grasp!");
    }

    // Acception step
    std::string acception = "y";
    std::cout << "Start trajectory test? (y, n)"; std::cin >> acception;
    if (acception != "y") return 1;

    youbotManipulator.moveToLineTrajectory(startPose, endPose);
    ros::Duration(deltaTime-0.1).sleep();
    //take object
    youbotManipulator.moveGripper(0.0115);
    //return to initial position
    youbotManipulator.moveToLineTrajectory(endPose, startPose);

}