#include <manipulation_control_node/ManipulationControlNode.h>
#include <manipulation_control_node/GraspingArmPositions.h>

#include <std_srvs/Empty.h>
#include <arm_kinematics/PoseArray.h>
#include <arm_kinematics/ManipulatorPose.h>
#include <cvision/CameraListObjects.h>

ManipulationControlNode::ManipulationControlNode(ros::NodeHandle n) : nh(n)
{
    ROS_INFO_STREAM("[Manipulation Control] Load Manipulation Control Node...");

    nh.param("/move_by_camera/camera_offset_x", cameraOffsetX, 0.051);
    nh.param("/move_by_camera/camera_offset_y", cameraOffsetY, 0.0048);
    nh.param("/move_by_camera/camera_offset_z", cameraOffsetZ, 0.0);

    ROS_INFO_STREAM("[Manipulation Control] ServiceClient: /get_list_objects...");
    getListObjectsClient = nh.serviceClient<cvision::CameraListObjects>("/get_list_objects");

    ROS_INFO_STREAM("[Manipulation Control] ServiceClient: /grasp_poses...");
    graspPosesClient = nh.serviceClient<arm_kinematics::PoseArray>("/grasp_poses");

    ROS_INFO_STREAM("[Manipulation Control] ServiceClient: /manipulator_pose...");
    moveToPoseClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/manipulator_pose");

    ROS_INFO_STREAM("[Manipulation Control] ServiceServer: /table_feasible...");
    tableServer = nh.advertiseService("/table_feasible", &ManipulationControlNode::execute, this);

    /////////////////////// INITIAL POSE FOR RECOGNIZED
    initialPoseForRecognized.position(0) = 0.3;
    initialPoseForRecognized.position(1) = 0.0;
    initialPoseForRecognized.position(2) = 0.5;

    /////////////////////// FIRST CONTAINER POINT
    firstContainerPoint.position(0) = -0.3;
    firstContainerPoint.position(1) = 0;
    firstContainerPoint.position(2) = -0.005;
    firstContainerPoint.orientation(0) = 0;
    firstContainerPoint.orientation(1) = 0;
    firstContainerPoint.orientation(2) = -3.1415;

    /////////////////////// SECOND CONTAINER POINT
    secondContainerPoint.position(0) = -0.25;
    secondContainerPoint.position(1) = -0.1;
    secondContainerPoint.position(2) = -0.005;
    secondContainerPoint.orientation(0) = 0;
    secondContainerPoint.orientation(1) = 0;
    secondContainerPoint.orientation(2) = -3.1415;

    /////////////////////// THIRD CONTAINER POINT
    thirdContainerPoint.position(0) = -0.25;
    thirdContainerPoint.position(1) = 0.1;
    thirdContainerPoint.position(2) = -0.005;
    thirdContainerPoint.orientation(0) = 0;
    thirdContainerPoint.orientation(1) = 0;
    thirdContainerPoint.orientation(2) = -3.1415;

    std::vector<bool> objectContaind = {false, false, false};
    std::vector<Pose> containersPose = {firstContainerPoint, secondContainerPoint, thirdContainerPoint};
    objectContainer = std::make_pair(objectContaind, containersPose);

    // finding angle for camera vision manipulator position
    std::vector<JointValues> sol;
    solver.solveIK(initialPoseForRecognized, sol);
    currentJointAngles = sol[0];
}

ManipulationControlNode::~ManipulationControlNode()
{}

size_t ManipulationControlNode::checkContainerContents(std::pair<std::vector<bool>, std::vector<Pose>> & container)
{   
    for (size_t i = 0; i < 3; ++i) {
        if (!container.first[i]) return i;
    }
    return -1;
}
bool ManipulationControlNode::execute(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
    // Messages
    cvision::CameraListObjects cameraListObjects;
    Pose recognizedObjectPose;
    arm_kinematics::CertesianPose armPose;
    armPose.position.resize(3);
    armPose.orientation.resize(3);

    arm_kinematics::ManipulatorPose initPose;
    armPose.position[0] = initialPoseForRecognized.position(0);
    armPose.position[1] = initialPoseForRecognized.position(1);
    armPose.position[2] = initialPoseForRecognized.position(2);
    armPose.orientation[0] = initialPoseForRecognized.orientation(0);
    armPose.orientation[1] = initialPoseForRecognized.orientation(1);
    armPose.orientation[2] = initialPoseForRecognized.orientation(2);
    initPose.request.pose = armPose;

    ROS_INFO_STREAM("Search free container.");
    size_t containerNumber = checkContainerContents(objectContainer);
    ROS_INFO_STREAM("Container number: " << containerNumber);
    
    while (containerNumber != -1 && nh.ok()) {
        arm_kinematics::PoseArray armPoseArray;

        if (moveToPoseClient.call(initPose)) ROS_INFO_STREAM("Go to initial pose.");
        else {
            ROS_FATAL_STREAM("Cant go to initial position");
            return false;
        }
        ros::Duration(2).sleep();

        ROS_INFO_STREAM("Turn ON camera.");
        ROS_INFO_STREAM("Reading data from camera.");
        do {
            getListObjectsClient.call(cameraListObjects);
        } while (cameraListObjects.response.list.empty() && nh.ok());

        recognizedObjectPose.position(0) = cameraListObjects.response.list[0].coordinates_center_frame[0] + cameraOffsetX;
        recognizedObjectPose.position(1) = cameraListObjects.response.list[0].coordinates_center_frame[1] + cameraOffsetY;
        recognizedObjectPose.position(2) = cameraListObjects.response.list[0].coordinates_center_frame[2] + cameraOffsetZ;
        recognizedObjectPose.orientation(0) = 0;
        recognizedObjectPose.orientation(1) = cameraListObjects.response.list[0].orientation[1];
        recognizedObjectPose.orientation(2) = 3.1415;

        ROS_INFO_STREAM("[Control Node] Recognized object position: (" 
            << recognizedObjectPose.position(0) << ", "
            << recognizedObjectPose.position(1) << ", " 
            << recognizedObjectPose.position(2) << ")\t"
            << "angle: " << recognizedObjectPose.orientation(1));

        Vector3d objectoPoseFromBase = solver.transformFromFrame5ToFrame0(currentJointAngles, recognizedObjectPose.position);
        recognizedObjectPose.position = objectoPoseFromBase;

        // TO DO add object height
        for (size_t i = 0; i < 3; ++i) {
            armPose.position[i] = recognizedObjectPose.position(i);
            armPose.orientation[i] = recognizedObjectPose.orientation(i);
        }
        armPoseArray.request.pose_array.push_back(armPose);
        for (size_t i = 0; i < 3; ++i) {
            armPose.position[i] = objectContainer.second[containerNumber].position(i);
            armPose.orientation[i] = objectContainer.second[containerNumber].orientation(i);
        }
        armPoseArray.request.pose_array.push_back(armPose);

        ROS_INFO_STREAM("Grasp the object.");
        if (graspPosesClient.call(armPoseArray)) ROS_INFO_STREAM("Turn ON camera.");
        else {
            ROS_FATAL_STREAM("Cant got to camera position.");
            return false;
        }
        objectContainer.first[containerNumber] = true;

        ROS_INFO_STREAM("Search free container.");
        containerNumber = checkContainerContents(objectContainer);
        ROS_INFO_STREAM("Container number: " << containerNumber);
    }
    return true;
}
void ManipulationControlNode::start() {

    while (nh.ok()) {
        ros::spin();
    }

}