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

#include <red_msgs/DestAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int numberOfContainers = 3;
double currGraspStep, currPlaceStep;
double initialPosPickAndPlace = currPlaceStep = currGraspStep = 0.2;
double step = 0.1;
double distance = 0.47;
double tableHeight = 0;
ArmKinematics solver;
arm_kinematics::CertesianPose armPose;
arm_kinematics::ManipulatorPose manipulatorPose;

size_t containerFilling(const std::vector<std::string> & containerObjectsName) {
    size_t count = 0;
    for (size_t i = 0; i < numberOfContainers; ++i) {
        if (containerObjectsName[i] == "") ++count;
    }
    return count;
}

size_t findObject(const std::vector<std::string> & containerObjectsName, std::string objectName) {
    size_t count = 0;
    for (size_t i = 0; i < numberOfContainers; ++i) {
        if (containerObjectsName[i] == objectName) {
            ROS_INFO_STREAM("Container " << containerObjectsName[i]);
            return i;
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasping_test");
    ros::NodeHandle nh;

    /* INITIALIZING */
    double cameraOffsetX, cameraOffsetY, cameraOffsetZ;
    Pose initialPoseForRecognized;
    Pose firstContainerPoint;
    Pose secondContainerPoint;
    Pose thirdContainerPoint;


    ROS_INFO_STREAM("[Grasp T] Intiializing");

    nh.getParam("/move_by_camera/camera_offset_x", cameraOffsetX);
    nh.getParam("/move_by_camera/camera_offset_y", cameraOffsetY);
    nh.getParam("/move_by_camera/camera_offset_z", cameraOffsetZ);
    ROS_INFO_STREAM("[Grasp T] Camera offset: " << cameraOffsetX << ", " << cameraOffsetY <<  ", " << cameraOffsetZ);

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /camera_task...");
    ros::ServiceClient cameraTaskClient = nh.serviceClient<red_msgs::CameraTask>("/camera_task");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /camera_stop...");
    ros::ServiceClient cameraStopClient = nh.serviceClient<red_msgs::CameraStop>("/camera_stop");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /grasp_object...");
    ros::ServiceClient graspObjectClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/grasp_object");

    ROS_INFO_STREAM("[Grasp T] ServiceClient: /manipulator_pose...");
    ros::ServiceClient moveToPoseClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/manipulator_pose");

    actionlib::SimpleActionClient<red_msgs::DestAction> naviAc("navi", true);

    // ROS_INFO_STREAM("[Grasp T] ServiceClient: /get_range...");
    // ros::ServiceClient rangefinderClient = nh.serviceClient<red_msgs::GetRange>("/get_range");

    /////////////////////// INITIAL POSE FOR RECOGNIZED
    initialPoseForRecognized.position(0) = 0.2;
    initialPoseForRecognized.position(1) = 0.0;
    initialPoseForRecognized.position(2) = 0.6;

    JointValues initialJV;
    solver.solveFullyIK(initialPoseForRecognized, initialJV);

    /////////////////////// FIRST CONTAINER POINT
    firstContainerPoint.position(0) = -0.18;
    firstContainerPoint.position(1) = -0.1;
    firstContainerPoint.position(2) = 0.015;
    firstContainerPoint.orientation(0) = 0;
    firstContainerPoint.orientation(1) = 0.5;
    firstContainerPoint.orientation(2) = -3.1415;

    /////////////////////// SECOND CONTAINER POINT
    secondContainerPoint.position(0) = -0.19;
    secondContainerPoint.position(1) = 0.0;
    secondContainerPoint.position(2) = 0.015;
    secondContainerPoint.orientation(0) = 0;
    secondContainerPoint.orientation(1) = 0;
    secondContainerPoint.orientation(2) = -3.1415;

    /////////////////////// THIRD CONTAINER POINT
    thirdContainerPoint.position(0) = -0.18;
    thirdContainerPoint.position(1) = 0.1;
    thirdContainerPoint.position(2) = 0.015;
    thirdContainerPoint.orientation(0) = 0;
    thirdContainerPoint.orientation(1) = -0.5;
    thirdContainerPoint.orientation(2) = -3.1415;

    std::vector<std::string> objectsName = {"", "", ""};
    std::vector<Pose> containersPose = {firstContainerPoint, secondContainerPoint, thirdContainerPoint};
    std::pair<std::vector<std::string>, std::vector<Pose>> objectContainer = std::make_pair(objectsName, containersPose);

    // heightTrasholdKoeff = 1/3;
    // openGripperWidth = 0.077;

    std::string answer;
    std::cout << "Start grasping test? (y, n): "; std::cin >> answer;

    /* EXECUTION */

    int graspedObjects = 3;
    int placedObjects = 3;
    if (answer == "y") {

        if (graspedObjects != 0) {
            // Messages
            red_msgs::CameraTask task;
            red_msgs::GetRange range;
            Pose recognizedObjectPose;
            std::vector<Pose> graspPoses;
            armPose.position.resize(3);
            armPose.orientation.resize(3);
            // TODO delete kostil!!!
            int objectNumber = 0;
            int graspingObjectNumber = 0;
            bool objectCapture = false;
            double miniumCoordinate = 100;
            size_t containerSize = containerFilling(objectContainer.first);
            if (containerSize == 0) {
                ROS_FATAL_STREAM("Containers is fill!");
                return false;
            }

            // Kostil
            // geometry_msgs::Pose2D desiredShiftOfBase;
            // red_msgs::DestGoal naviAcGoal;

            // desiredShiftOfBase.x = 0;
            // desiredShiftOfBase.y = currGraspStep;
            // desiredShiftOfBase.theta = 0;
            // naviAcGoal.task = "dist";
            // naviAcGoal.dist = desiredShiftOfBase;
            // naviAc.sendGoal(naviAcGoal);
            // naviAc.waitForResult();
            // actionlib::SimpleClientGoalState state_ac = naviAc.getState();


            // if(state_ac.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //     ROS_INFO("[GTP] Start goal is successfully processed");
            // }
            
            ROS_INFO_STREAM("Search free container.");
            size_t containerNumber = 0;
            ROS_INFO_STREAM("Container number: " << containerNumber);

            while (containerSize != numberOfContainers - graspedObjects && nh.ok()) {
                if (currGraspStep == -initialPosPickAndPlace) {
                    currGraspStep = initialPosPickAndPlace;
                }

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

                // if (rangefinderClient.call(range)) ROS_INFO_STREAM("Measuring distance.");
                // else {
                //     ROS_WARN_STREAM("Cant read data from rangefinder.");
                // }
                // distance = rf.getRange();
                // ROS_INFO_STREAM("Distance : " << distance);

                ROS_INFO_STREAM("Turn ON camera.");
                ROS_INFO_STREAM("Reading data from camera.");
                task.request.mode = 1;
                task.request.shape = "";
                task.request.distance = distance;
                if (cameraTaskClient.call(task)) {
                    // if (task.response.list.empty()) {
                    //     currGraspStep -= step;
                    //     desiredShiftOfBase.x = 0;
                    //     desiredShiftOfBase.y = currGraspStep;
                    //     desiredShiftOfBase.theta = 0;
                    //     naviAcGoal.task = "dist";
                    //     naviAcGoal.dist = desiredShiftOfBase;
                    //     naviAc.sendGoal(naviAcGoal);
                    //     naviAc.waitForResult();
                    //     state_ac = naviAc.getState();
                    // }
                }

                // TODO delete kostil
                for (objectNumber = 0; objectNumber < task.response.list.size(); ++objectNumber) {
                    // double objectHeight = heights[graspingObjectNumber];

                    recognizedObjectPose.position(0) = task.response.list[objectNumber].coordinates_center_frame[0] + cameraOffsetX;
                    recognizedObjectPose.position(1) = task.response.list[objectNumber].coordinates_center_frame[1] + cameraOffsetY;
                    recognizedObjectPose.position(2) = task.response.list[objectNumber].coordinates_center_frame[2] + cameraOffsetZ;
                    recognizedObjectPose.orientation(0) = 0;
                    recognizedObjectPose.orientation(1) = task.response.list[objectNumber].orientation[1];
                    recognizedObjectPose.orientation(2) = 3.1415;

                    ROS_INFO_STREAM("[Control Node] Recognized object position: (" 
                        << recognizedObjectPose.position(0) << ", "
                        << recognizedObjectPose.position(1) << ", " 
                        << recognizedObjectPose.position(2) << ")\t"
                        << "angle: " << recognizedObjectPose.orientation(1));

                    Vector3d objectoPoseFromBase = solver.transformFromFrame5ToFrame0(initialJV, recognizedObjectPose.position);
                    recognizedObjectPose.position = objectoPoseFromBase;
                    if (tableHeight == 0) tableHeight = recognizedObjectPose.position(2);

                    graspPoses.push_back(recognizedObjectPose);
                }

                // Find minimum of Y coordinate
                Pose currentGraspPose;
                for (int i = 0; i < graspPoses.size(); ++i) {
                    if (miniumCoordinate > graspPoses[i].position(2)) {
                        miniumCoordinate = graspPoses[i].position(2);
                        currentGraspPose = graspPoses[i];
                    }
                }
                miniumCoordinate = 100;

                if (task.response.list.size() != 0) {
                    
                    // if (abs(currentGraspPose.position(1)) > 0.1) {

                    //     desiredShiftOfBase.x = 0;
                    //     desiredShiftOfBase.y = currentGraspPose.position(1);
                    //     desiredShiftOfBase.theta = 0;
                    //     naviAcGoal.task = "dist";
                    //     naviAcGoal.dist = desiredShiftOfBase;
                    //     naviAc.waitForResult();
                    //     state_ac = naviAc.getState();
                    //     if(state_ac.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    //         ROS_INFO("[GTP] Goal is successfully processed");
                    //     }
                    //     break;
                    // }

                    // Grasp object
                    for (size_t i = 0; i < 3; ++i) {
                        armPose.position[i] = recognizedObjectPose.position(i);
                        armPose.orientation[i] = recognizedObjectPose.orientation(i);
                    }
                    manipulatorPose.request.pose = armPose;
                    manipulatorPose.request.task = 1;   // Grasp task
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
                    manipulatorPose.request.pose.position[2] += 0.01;
                    manipulatorPose.request.task = 2;   // Put task

                    ROS_INFO_STREAM("Put the object.");
                    if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Turn ON camera.");
                    else {
                        ROS_FATAL_STREAM("Cant got to camera position.");
                        return false;
                    }
                    objectContainer.first[containerNumber] = "s";

                    ROS_INFO_STREAM("Search free container.");
                    ++containerNumber;
                    ROS_INFO_STREAM("Container number: " << containerNumber);
                    containerSize--;

                    // ++graspingObjectNumber;
                    // objectCapture = true;
                } else break;
            }
        }

        ROS_WARN_STREAM("End working");
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

        if (placedObjects != 0) {

            armPose.position.resize(3);
            armPose.orientation.resize(3);

            size_t containerSize = containerFilling(objectContainer.first);
            size_t container;
            double currentY = 0;

            double experimentalX = 0.32;

            container = findObject(objectContainer.first, "s");
            while (containerSize != numberOfContainers && nh.ok()) {

                objectContainer.first[container] = "";
                ROS_INFO_STREAM("number " << container);
                ROS_INFO_STREAM("first: " << "( " << objectContainer.first[0] << ", " 
                    << objectContainer.first[1] << ", " << objectContainer.first[2] << ")");

                // Grasp object from container
                for (size_t i = 0; i < 3; ++i) {
                    armPose.position[i] = objectContainer.second[container].position(i);
                    armPose.orientation[i] = objectContainer.second[container].orientation(i);
                }
                manipulatorPose.request.pose = armPose;
                manipulatorPose.request.task = 1;   // Grasp task
                ROS_INFO_STREAM("Grasp the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }

                // Place object to table
                armPose.position[0] = experimentalX;
                armPose.position[1] = currentY;
                armPose.position[2] = tableHeight; // 0.6 - initial obj. coordinate z

                armPose.orientation[0] = 0;
                armPose.orientation[1] = 0;
                armPose.orientation[2] = 3.1415;

                manipulatorPose.request.pose = armPose;
                manipulatorPose.request.task = 2;
                ROS_INFO_STREAM("Grasp the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }
                --containerSize;
                currentY -= step;
                container = findObject(objectContainer.first, "s");
            }
        }

    }
}
