#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>
#include <iostream>
#include <string>

#include <std_srvs/Empty.h>
#include <arm_kinematics/PoseArray.h>
#include <arm_kinematics/ManipulatorPose.h>
#include <red_msgs/CameraTask.h>
#include <red_msgs/CameraStop.h>
#include <red_msgs/ManipulationObjects.h>
#include <red_msgs/GetRange.h>

#include <manipulation_control_node/GraspingArmPositions.h>

size_t checkContainerContents(std::pair<std::vector<bool>, std::vector<Pose> > & container)
{   
    for (size_t i = 0; i < 3; ++i) {
        if (!container.first[i]) return i;
    }
    return -1;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasping_test");
    ros::NodeHandle nh;

    /* INITIALIZING */
    double cameraOffsetX, cameraOffsetY, cameraOffsetZ;

    ROS_INFO_STREAM("[Grasp T] Intiializing");

    nh.getParam("/grasping_test/camera_offset_x", cameraOffsetX);
    nh.getParam("/grasping_test/camera_offset_y", cameraOffsetY);
    nh.getParam("/grasping_test/camera_offset_z", cameraOffsetZ);
    ROS_INFO_STREAM("[Grasp T] Camera offset: " << cameraOffsetX << ", " << cameraOffsetY <<  ", " << cameraOffsetZ);

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /camera_task...");
    ros::ServiceClient cameraTaskClient = nh.serviceClient<red_msgs::CameraTask>("/camera_task");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /camera_stop...");
    ros::ServiceClient cameraStopClient = nh.serviceClient<red_msgs::CameraStop>("/camera_stop");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /grasp_object...");
    ros::ServiceClient graspObjectClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/grasp_object");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /manipulator_pose...");
    ros::ServiceClient moveToPoseClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/manipulator_pose");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /get_range...");
    ros::ServiceClient rangefinderClient = nh.serviceClient<red_msgs::GetRange>("/get_range");

    /////////////////////// INITIAL POSE FOR RECOGNIZED
    initialPoseForRecognized.position(0) = 0.3;
    initialPoseForRecognized.position(1) = 0.0;
    initialPoseForRecognized.position(2) = 0.6;

    /////////////////////// FIRST CONTAINER POINT
    firstContainerPoint.position(0) = -0.35;
    firstContainerPoint.position(1) = 0;
    firstContainerPoint.position(2) = -0.02;
    firstContainerPoint.orientation(0) = 0;
    firstContainerPoint.orientation(1) = 0;
    firstContainerPoint.orientation(2) = -3.1415;

    /////////////////////// SECOND CONTAINER POINT
    secondContainerPoint.position(0) = -0.35;
    secondContainerPoint.position(1) = 0.1;
    secondContainerPoint.position(2) = -0.02;
    secondContainerPoint.orientation(0) = 0;
    secondContainerPoint.orientation(1) = 0;
    secondContainerPoint.orientation(2) = -3.1415;

    /////////////////////// THIRD CONTAINER POINT
    thirdContainerPoint.position(0) = -0.35;
    thirdContainerPoint.position(1) = -0.1;
    thirdContainerPoint.position(2) = -0.02;
    thirdContainerPoint.orientation(0) = 0;
    thirdContainerPoint.orientation(1) = 0;
    thirdContainerPoint.orientation(2) = -3.1415;

    std::vector<bool> objectContaind = {false, false, false};
    std::vector<Pose> containersPose = {firstContainerPoint, secondContainerPoint, thirdContainerPoint};
    std::pair<std::vector<bool>, std::vector<Pose>> objectContainer = std::make_pair(objectContaind, containersPose);

    std::string answer;
    std::cout << "Start grasping test? (y, n): "; std::cin >> answer;

    /* EXECUTION */

    if (answer == "y") {

        // Messages
        red_msgs::CameraTask task;
        red_msgs::GetRange range;
        Pose recognizedObjectPose;
        arm_kinematics::CertesianPose armPose;
        armPose.position.resize(3);
        armPose.orientation.resize(3);
        ArmKinematics solver;
        JointValues currentJointAngles;

        double distance = 0;

        ROS_INFO_STREAM("Search free container.");
        size_t containerNumber = checkContainerContents(objectContainer);
        ROS_INFO_STREAM("Container number: " << containerNumber);

        while (containerNumber != -1 && nh.ok()) {

            arm_kinematics::ManipulatorPose manipulatorPose;
            armPose.position[0] = initialPoseForRecognized.position(0);
            armPose.position[1] = initialPoseForRecognized.position(1);
            armPose.position[2] = initialPoseForRecognized.position(2);
            armPose.orientation[0] = initialPoseForRecognized.orientation(0);
            armPose.orientation[1] = initialPoseForRecognized.orientation(1);
            armPose.orientation[2] = initialPoseForRecognized.orientation(2);
            manipulatorPose.request.pose = armPose;

            if (moveToPoseClient.call(manipulatorPose)) ROS_INFO_STREAM("Go to initial pose.");
            else {
                ROS_FATAL_STREAM("Cant go to initial position");
                return false;
            }
            ros::Duration(2).sleep();
            solver.solveFullyIK(initialPoseForRecognized, currentJointAngles);

            if (rangefinderClient.call(range)) ROS_INFO_STREAM("Measuring distance.");
            else {
                ROS_WARN_STREAM("Cant read data from rangefinder.");
            }
            // distance = rf.getRange();
            distance = range.response.distance;
            ROS_INFO_STREAM("Distance : " << distance);

            ROS_INFO_STREAM("Turn ON camera.");
            ROS_INFO_STREAM("Reading data from camera.");
            task.request.mode = 1;
            task.request.shape = "";
            task.request.distance = distance;
            do {
                cameraTaskClient.call(task);
            } while (task.response.list.empty() && nh.ok());

            recognizedObjectPose.position(0) = task.response.list[0].coordinates_center_frame[0] + cameraOffsetX;
            recognizedObjectPose.position(1) = task.response.list[0].coordinates_center_frame[1] + cameraOffsetY;
            recognizedObjectPose.position(2) = task.response.list[0].coordinates_center_frame[2] + cameraOffsetZ;
            recognizedObjectPose.orientation(0) = 0;
            recognizedObjectPose.orientation(1) = task.response.list[0].orientation[1];
            recognizedObjectPose.orientation(2) = 3.1415;

            ROS_INFO_STREAM("[Control Node] Recognized object position: (" 
                << recognizedObjectPose.position(0) << ", "
                << recognizedObjectPose.position(1) << ", " 
                << recognizedObjectPose.position(2) << ")\t"
                << "angle: " << recognizedObjectPose.orientation(1));

            Vector3d objectoPoseFromBase = solver.transformFromFrame5ToFrame0(currentJointAngles, recognizedObjectPose.position);
            recognizedObjectPose.position = objectoPoseFromBase;

            // TO DO add object height
            // Grasp object
            for (size_t i = 0; i < 3; ++i) {
                armPose.position[i] = recognizedObjectPose.position(i);
                armPose.orientation[i] = recognizedObjectPose.orientation(i);
            }
            manipulatorPose.request.pose = armPose;
            ROS_INFO_STREAM("Grasp the object.");
            if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
            else {
                ROS_FATAL_STREAM("Cant got to camera position.");
                return false;
            }

            // Put object
            for (size_t i = 0; i < 3; ++i) {
                armPose.position[i] = objectContainer.second[containerNumber].position(i);
                armPose.orientation[i] = objectContainer.second[containerNumber].orientation(i);
            }
            manipulatorPose.request.pose = armPose;

            ROS_INFO_STREAM("Put the object.");
            if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Turn ON camera.");
            else {
                ROS_FATAL_STREAM("Cant got to camera position.");
                return false;
            }
            objectContainer.first[containerNumber] = true;

            ROS_INFO_STREAM("Search free container.");
            containerNumber = checkContainerContents(objectContainer);
            ROS_INFO_STREAM("Container number: " << containerNumber);

        }
    }
}