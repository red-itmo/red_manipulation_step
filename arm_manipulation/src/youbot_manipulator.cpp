#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

YoubotManipulator::YoubotManipulator(ros::NodeHandle & nodeHandle)
    :nh(nodeHandle)
{
    nh.param("youBotDriverCycleFrequencyInHz", lr, 100.0);
    this->maxAccel = 0.1;
    this->maxVel = 0.05;
    this->timeStep = 0.05;
    initArmTopics();
}

YoubotManipulator::~YoubotManipulator() {}

void YoubotManipulator::initArmTopics()
{
    ROS_INFO_STREAM("[Arm Manipulation] Load YouBot Arm Manipulation..." << "\n");
    ROS_INFO_STREAM("[Arm Manipulation] Publisher: arm_1/arm_controller/position_command...");
    armPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);

    ROS_INFO_STREAM("[Arm Manipulation] Publisher: arm_1/gripper_controller/position_command...");
    gripperPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/gripper_controller/position_command", 1);

    ROS_INFO_STREAM("[Arm Manipulation] Subsciber: arm_1/joint_states...");
    stateSubscriber = nh.subscribe("/arm_1/joint_states", 10, &YoubotManipulator::stateCallback, this);

    ROS_INFO_STREAM("[Arm Manipulation] Load ActionClient arm_1/arm_controller/velocity_joint_trajecotry");

    trajectoryAC = new ActionClent ("arm_1/arm_controller/velocity_joint_trajecotry", true);
}

void YoubotManipulator::setConstraints(const double maxAccel, const double maxVel, const double timeStep)
{
    this->maxAccel = maxAccel;
    this->maxVel = maxVel;
    this->timeStep = timeStep;
    ROS_INFO_STREAM("[Arm Manipulation] TRJ parameters:"
        << " Driver Rate: " << lr
        << " Max Vel.: " << maxVel
        << " Max Accel.: " << maxAccel);
}

bool YoubotManipulator::moveArm(const JointValues & angles)
{
    ROS_INFO("[Arm Manipulation] moveArm");
    brics_actuator::JointPositions jointPositions;
    jointPositions = createArmPositionMsg(angles);
    //wait a little bit until subscibers connect
    ros::Duration(0.5).sleep();
    if(armPublisher.getNumSubscribers()==0)
        ROS_WARN("[arm_manipulation] No subscibers are connected to armPublisher...");
    armPublisher.publish(jointPositions);

    if (!checkAchievementOfPosition(angles)) {
        ROS_WARN("Position is not desired!");
        return false;
    }
    return false;
}

bool YoubotManipulator::moveArm(const Pose & pose)
{
    brics_actuator::JointPositions jointPositions;
    JointValues jointAngles;
    Vector3d zeros, pos;
    ROS_INFO_STREAM("[Arm Manipulation] Position: (" << pose.position(0) << " " << pose.position(1) << " " << pose.position(2) << ")");
    ROS_INFO_STREAM("[Arm Manipulation] Orientation: (" << pose.orientation(0) << " " << pose.orientation(1) << " " << pose.orientation(2) << ")");

    if (solver.solveFullyIK(pose, jointAngles)) {
        // Convert joint values
        // pos = solver.transformFromFrame5ToFrame0(jointAngles, zeros);
        // ROS_INFO_STREAM("Forw. Kin. Pos.: (" << pos(0) << ", " << pos(1) << ", " << pos(2) << ")");
        makeYoubotArmOffsets(jointAngles);
        return moveArm(jointAngles);
    }
    else{
        ROS_ERROR_STREAM("[Arm Manipulation] Solution NOT found!");
        return false;
    }
}

void YoubotManipulator::moveGripper(double jointValue)
{
    brics_actuator::JointPositions jointPositions = createGripperPositionMsg(jointValue);
    gripperPublisher.publish(jointPositions);
}


brics_actuator::JointPositions createArmPositionMsg(const JointValues & jointAngles)
{
    //ROS_INFO("createArmPositionMsg...");
    brics_actuator::JointPositions jointPositions;
    brics_actuator::JointValue jointValue;
    for (size_t i = 0; i < 5; ++i) {
        jointValue.timeStamp = ros::Time::now();
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        jointValue.joint_uri = jointName.str();
        jointValue.unit = "rad";
        jointValue.value = jointAngles(i);
        jointPositions.positions.push_back(jointValue);
    }
    return jointPositions;
}

brics_actuator::JointPositions createGripperPositionMsg(double jointValue)
{
    brics_actuator::JointPositions jointPositions;

    brics_actuator::JointValue joint;
    joint.timeStamp = ros::Time::now();
    joint.unit = "m";

    joint.value = jointValue;
    joint.joint_uri = "gripper_finger_joint_l";
    jointPositions.positions.push_back(joint);

    joint.value = jointValue;
    joint.joint_uri = "gripper_finger_joint_r";
    jointPositions.positions.push_back(joint);

    return jointPositions;
}

bool YoubotManipulator::checkAchievementOfPosition(const JointValues & desiredValues) {
    ROS_INFO("[Arm Manipulation]Checking Achievement Of Position");
    JointValues currentValues, diff, prevValues;
    bool notChange = true;
    uint32_t duration, startTimeAchievement = ros::Time::now().sec;
    uint32_t WATCHDOG_TIME = 8;
    do {
        currentValues = stateValues;
        do {
            ros::spinOnce();
            for (size_t i = 0; i < DOF; ++i) {
                notChange = notChange && (currentValues(i) == stateValues(i));
            }
            //std::cout<<"."<<notChange;
        } while(notChange && nh.ok());

        duration = ros::Time::now().sec - startTimeAchievement;
        if (duration>=WATCHDOG_TIME)
        {
            ROS_WARN("WATCHDOG timeout, inaccurate position!");
            return false;
        }
        diff = desiredValues - currentValues;
    } while (diff.norm() > 0.1 && nh.ok());
    return true;
}

void YoubotManipulator::stateCallback(const sensor_msgs::JointStatePtr & msg) {
    for (size_t i = 0; i < DOF; ++i) {
        stateValues(i) = msg->position[i];
    }
}

bool YoubotManipulator::goToPose(red_msgs::ArmPoses::Request & req, red_msgs::ArmPoses::Response & res)
{
    ROS_DEBUG_NAMED("arm_manipulation","[Arm Manipulation] goToPose");
    Pose pose;

    if (req.poses.size() != 1) {
        ROS_ERROR("Size of input array is NOT VALID");
        return false;
    }

    pose.position(0) = req.poses[0].x;
    pose.position(1) = req.poses[0].y;
    pose.position(2) = req.poses[0].z;
    pose.orientation(1) = req.poses[0].theta;
    pose.orientation(2) = req.poses[0].psi;

    res.error = moveArm(pose);
    return true;
}

void YoubotManipulator::moveArmLoop()
{
    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /MoveLine...");
    trajectoryServer = nh.advertiseService("MoveLine", &YoubotManipulator::trajectoryMove, this);

    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /manipulator_pose...");
    poseServer = nh.advertiseService("manipulator_pose", &YoubotManipulator::goToPose, this);

    while(nh.ok()) {
        ros::spin();
    }
}

bool YoubotManipulator::serviceTurnOnMotors(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    JointValues initAngles;
    initAngles.setAll(0.1);
    initAngles(2)=-0.1;
    initAngles(4)=0.2;
    moveArm(initAngles);

    std_srvs::Empty empty;
    ros::service::call("arm_1/switchOnMotors", empty);
    ros::service::call("base/switchOnMotors", empty);
    return true;
}

bool YoubotManipulator::goToInitAndRelax()
{
    setConstraints(0.05,0.05,0.2);
    JointValues initAngles;
    initAngles.setAll(0.1);
    initAngles(2)=-0.1;
    initAngles(4)=0.2;
    moveArm(initAngles);
    //kill Motors
    std_srvs::Empty empty;
    ros::service::call("arm_1/switchOffMotors", empty);
    ros::service::call("base/switchOffMotors", empty);
    ROS_INFO("Motors turned off!");

    ros::spinOnce();

    ros::ServiceClient srvRelaxMotorsClient = nh.serviceClient<std_srvs::Empty>("switchOnMotors", false);
    ROS_INFO_STREAM("[Arm Manipulation] To turn motors on call /switchOnMotors...");
    srvRelaxMotorsServer = nh.advertiseService("switchOnMotors", &YoubotManipulator::serviceTurnOnMotors, this);

    ros::spin();
}