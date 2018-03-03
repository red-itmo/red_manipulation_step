#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

YoubotManipulator::YoubotManipulator(ros::NodeHandle & nodeHandle)
    :nh(nodeHandle)
{
    nh.param("/trajectory_test/youBotDriverCycleFrequencyInHz", lr, 100.0);
    nh.param("simulation", sim, true);
    nh.param("/trajectory_test/timeStep", timeStep, 0.05);
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
}

void YoubotManipulator::initActionClient(const double aMax, const double vMax, const double timeStep)
{
    maxAccel = aMax;
    maxVel = vMax;
    this->timeStep = timeStep;

    ROS_INFO_STREAM("[Arm Manipulation] Load ActionClient arm_1/arm_controller/velocity_joint_trajecotry");
    trajectoryAC = new ActionClent("arm_1/arm_controller/velocity_joint_trajecotry", true);

    ROS_INFO_STREAM("[Arm Manipulation] TRJ parameters:"
        << " Driver Rate: " << lr
        << " Max Vel.: " << maxVel
        << " Max Accel.: " << maxAccel);
}

bool YoubotManipulator::moveArm(const JointValues & angles) 
{
    ROS_INFO("[Arm Manipulation] moveArm");
    brics_actuator::JointPositions jointPositions;
    if (checkAngles(angles)) {
        jointPositions = createArmPositionMsg(angles);
        armPublisher.publish(jointPositions);
        // ros::Duration(0.5).sleep();
         
            if (!checkAchievementOfPosition(angles)) {
                ROS_WARN("Position is not desired!");
                return false;
            }
        return true;
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
            ROS_DEBUG_STREAM_ONCE("norm:"<<diff.norm()<<
            " 0:" << desiredValues(0)*180/M_PI<< ";"<<currentValues(0)*180/M_PI<<" "<<
         " 1:" << desiredValues(1)*180/M_PI<< ";"<<currentValues(1)*180/M_PI<<" "<< 
         " 2:" << desiredValues(2)*180/M_PI<< ";"<<currentValues(2)*180/M_PI<<" "<<
         " 3:" << desiredValues(3)*180/M_PI<< ";"<<currentValues(3)*180/M_PI<<
         " 4:" << desiredValues(4)*180/M_PI<< ";"<<currentValues(4)*180/M_PI);
            ROS_WARN("WATCHDOG timeout, inaccurate position!");
            return false;
        }
        diff = desiredValues - currentValues;
    } while (diff.norm() > 0.2 && nh.ok());
    return true;
}

void YoubotManipulator::stateCallback(const sensor_msgs::JointStatePtr & msg) {
    for (size_t i = 0; i < DOF; ++i) {
        stateValues(i) = msg->position[i];
    }
}
bool YoubotManipulator::goToPose(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res)
{
    ROS_DEBUG_NAMED("arm_manipulation","[Arm Manipulation] goToPose");
    Pose pose;
    pose.position(0) = req.pose.position[0];
    pose.position(1) = req.pose.position[1];
    pose.position(2) = req.pose.position[2];
    pose.orientation(0) = req.pose.orientation[0];
    pose.orientation(1) = req.pose.orientation[1];
    pose.orientation(2) = req.pose.orientation[2];
    
    res.feasible = moveArm(pose);
    return true;
}

bool YoubotManipulator::graspObject(const Pose & p)
{
    JointValues jointAngles;
    // TODO add dynamic object height
    double objectHeight = 0.05;

    // First Step --------
    Pose startPose = p;
    Pose endPose = startPose;

    startPose.position(2) += 0.05;
    if (!solver.solveFullyIK(startPose, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPose.position(0) << ", " << startPose.position(1)  << ", " << startPose.position(2));
        return false;
    }

    if (!solver.solveFullyIK(endPose, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (endPos): " << endPose.position(0) << ", " << endPose.position(1)  << ", " << endPose.position(2));
        return false;
    }

    moveArm(startPose);
    moveGripper(0.0115);
     
    ros::Duration(0.5).sleep();

    moveToLineTrajectory(startPose, endPose);
    moveGripper(0.0);
     
    ros::Duration(0.5).sleep();

    // move up
    moveToLineTrajectory(endPose, startPose);

    return true;
}

bool YoubotManipulator::putObject(const Pose & p)
{
    JointValues jointAngles;
    // TODO add dynamic object height
    double objectHeight = 0.05;

    // First Step --------
    Pose startPose = p;
    Pose endPose = startPose;

    startPose.position(2) += 0.05;
    if (!solver.solveFullyIK(startPose, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPose.position(0) << ", " << startPose.position(1)  << ", " << startPose.position(2));
        return false;
    }

    if (!solver.solveFullyIK(endPose, jointAngles)) {
        ROS_FATAL_STREAM("Solution is not found (endPos): " << endPose.position(0) << ", " << endPose.position(1)  << ", " << endPose.position(2));
        return false;
    }

    moveArm(startPose);
     
    ros::Duration(0.5).sleep();

    moveToLineTrajectory(startPose, endPose);
    moveGripper(0.0115);
     
    ros::Duration(0.5).sleep();

    moveToLineTrajectory(endPose, startPose);
    return true;
}

void YoubotManipulator::moveArmLoop()
{
    nh.param("/arm_manipulation/max_Vel", maxVel, 0.03);
    if (!nh.hasParam("/arm_manipulation/max_vel"))
      {
        ROS_WARN("[Arm Manipulation]No param '/arm_manipulation/max_vel', setting 0.03");
      }

    nh.param("/arm_manipulation/max_Accel", maxAccel, 0.1);
    if (!nh.hasParam("/arm_manipulation/max_accel"))
      {
        ROS_WARN("[Arm Manipulation]No param '/arm_manipulation/max_Accel', setting 0.1");
      }

    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /grasp_object...");
    trajectoryServer = nh.advertiseService("grasp_object", &YoubotManipulator::trajectoryMove, this);

    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /manipulator_pose...");
    poseServer = nh.advertiseService("manipulator_pose", &YoubotManipulator::goToPose, this);

    initArmTopics();
    initActionClient(maxAccel, maxVel, timeStep);
    
    while(nh.ok()) {
        ros::spin();
    }
}
