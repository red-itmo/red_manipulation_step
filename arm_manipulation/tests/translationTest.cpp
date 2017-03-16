#include <arm_kinematics/ArmKinematics.h>
#include <arm_kinematics/CameraFrame.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

JointValues jointAngles;
Vector3d cameraFrameObjectPosition;

void stateCallback(const sensor_msgs::JointState msg)
{
    jointAngles(0) = msg.position[0];
    jointAngles(1) = msg.position[1];
    jointAngles(2) = msg.position[2];
    jointAngles(3) = msg.position[3];
    jointAngles(4) = msg.position[4];
}

bool getObjectPosition(arm_kinematics::CameraFrame::Request & req, arm_kinematics::CameraFrame::Response & res)
{
    ROS_INFO_STREAM("Object position: (" << req.position[0] << ", " << req.position[1] << ", " << req.position[2] << ")");
    cameraFrameObjectPosition(0) = req.position[0];
    cameraFrameObjectPosition(1) = req.position[1];
    cameraFrameObjectPosition(2) = req.position[2];
    res.state = 0;

    return true;
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "calibr_cam_pos");
    ros::NodeHandle nh;

    std::vector<double> initialObjectPosition;

    // Solver for search manipulator position and object position from camera.
    ArmKinematics solver;

    // Camera Offset
    double xOffset = 0, yOffset = 0, zOffset = 0;

    // Manipulator params from URDF model.
    double d0x = 0.024;
    double d0z = 0.096;
    double d1x = 0.033;
    double d1z = 0.019;
    double d2 = 0.155;
    double d3 = 0.135;
    double d4 = 0.13;

    // angle offset
    double offset1 = 2.9496064359;
    double offset2 = 1.1344640138;
    double offset3 = -2.5481807079;
    double offset4 = 1.7889624833;
    double offset5 = 2.9234264971;

    // Manipulator params
    double phi1 = 0, alpha = 0, phi5 = 0;

    // Plane manipulator
    double dx = 0, dz = 0;


    if (!nh.getParam("/calibr_cam_pos/object_position", initialObjectPosition)) {
        ROS_FATAL_STREAM("Parameter object_position is not found.");
        return 1;
    } else {
        ROS_INFO_STREAM("Initial object position: ("
            << initialObjectPosition[0]
            << ", " << initialObjectPosition[1]
            << ", " << initialObjectPosition[2] << ")");
    }

    // Data translaitors
    ros::Subscriber manipulatorStateSubscriber = nh.subscribe("/arm_1/joint_states", 1, stateCallback);
    ros::ServiceServer cameraServer = nh.advertiseService<arm_kinematics::CameraFrame::Request, arm_kinematics::CameraFrame::Response > ("object_position", getObjectPosition);

    jointAngles(0) = 0;
    jointAngles(1) = 0;
    jointAngles(2) = 0;
    jointAngles(3) = 0;
    jointAngles(4) = 0;

    // Read current joint angles and unsubscribe.
    while (jointAngles(0) == 0 && jointAngles(1) == 0 && jointAngles(2) == 0 && jointAngles(3) == 0 && jointAngles(4) == 0)
        ros::spinOnce();
    manipulatorStateSubscriber.shutdown();

    // Translate JointValues
    jointAngles(0) = offset1 - jointAngles(0);
    jointAngles(1) = jointAngles(1) - offset2;
    jointAngles(2) = jointAngles(2) - offset3;
    jointAngles(3) = jointAngles(3) - offset4;
    jointAngles(4) = offset5 - jointAngles(4);

    while (ros::ok()) {

        // Read position from camera
        ros::spinOnce();

        phi1 = jointAngles(0);
        alpha = jointAngles(1) + jointAngles(2) + jointAngles(3);
        phi5 = jointAngles(4);

        // Plane manipulator
        dx = d2*sin(jointAngles(1)) + d3*sin(jointAngles(1) + jointAngles(2)) + d4*sin(alpha) + d1x;
        dz = d2*cos(jointAngles(1)) + d3*cos(jointAngles(1) + jointAngles(2)) + d4*cos(alpha) + d1z;

        // Calculate shift
        xOffset = cos(phi5)*cos(alpha) * (sin(phi1)*initialObjectPosition[1] + cos(phi1)*(initialObjectPosition[0] - d0x) - dx) 
                - cos(phi5)*sin(alpha)*(initialObjectPosition[2] - dz - d0z)
                + sin(phi5) * (cos(phi1)*initialObjectPosition[1] - sin(phi1)*(initialObjectPosition[0] - d0x)) - cameraFrameObjectPosition(0);
        yOffset = cos(phi5)* (cos(phi1)*initialObjectPosition[1] - sin(phi1)*(initialObjectPosition[0] - d0x))
                - sin(phi5)* cos(alpha)*(sin(phi1)*initialObjectPosition[1] + cos(phi1)*(initialObjectPosition[0] - d0x) - dx)
                - sin(phi5)* sin(alpha)*(initialObjectPosition[2] - dz - d0z) - cameraFrameObjectPosition(1);
        zOffset = cos(alpha)*(initialObjectPosition[2] - dz - d0z)
                + sin(alpha)*(sin(phi1)*initialObjectPosition[1] + cos(phi1)*(initialObjectPosition[0] - d0x) - dx) - cameraFrameObjectPosition(2);

        // output
        // ...
        ROS_INFO_STREAM("offset: (" << xOffset << ", " << yOffset << ", " << zOffset << ")");

    }

    return 0;
}
