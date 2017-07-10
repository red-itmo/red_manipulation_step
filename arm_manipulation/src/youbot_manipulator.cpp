#include <arm_manipulation/youbot_manipulator.h>
#include <arm_kinematics/KinematicConstants.h>
#include <trajectory_generator/TrajectoryGenerator.h>

YoubotManipulator::YoubotManipulator(ros::NodeHandle & nodeHandle)
    :nh(nodeHandle)
{
    ROS_INFO_STREAM("[Arm Manipulation] Load YouBot Arm Manipulation..." << "\n");
    ROS_INFO_STREAM("[Arm Manipulation] Publisher: arm_1/arm_controller/position_command...");
    armPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);
    ROS_INFO_STREAM("[Arm Manipulation] Publisher: arm_1/gripper_controller/position_command...");
    gripperPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/gripper_controller/position_command", 1);

    // Reading variable for trajectory control
    nh.param("youBotDriverCycleFrequencyInHz", lr, 100.0);
    nh.param("/arm_manipulation/max_vel", maxVel, 0.03);
    nh.param("/arm_manipulation/max_accel", maxAccel, 0.1);

    sleep(1);
}

YoubotManipulator::~YoubotManipulator() {}

void YoubotManipulator::moveArm(const Pose & pose)
{
    brics_actuator::JointPositions jointPositions;
    JointValues jointAngles;

    ROS_INFO_STREAM("[Arm Manipulation] Position: (" << pose.position(0) << " " << pose.position(1) << " " << pose.position(2) << ")");
    ROS_INFO_STREAM("[Arm Manipulation] Orientation: (" << pose.orientation(0) << " " << pose.orientation(1) << " " << pose.orientation(2) << ")");
    
    if (solver.solveFullyIK(pose, jointAngles)) {
        // Convert joint values
        makeYoubotArmOffsets(jointAngles);
        jointPositions = createArmPositionMsg(jointAngles);
        ROS_INFO_STREAM("[Arm Manipulation] Sending command...");
        armPublisher.publish(jointPositions);
        ros::Duration(2).sleep();
    }
    else ROS_ERROR_STREAM("[Arm Manipulation] Solution NOT found!");
}

void YoubotManipulator::moveGripper(double jointValue)
{
    brics_actuator::JointPositions jointPositions = createGripperPositionMsg(jointValue); 
    gripperPublisher.publish(jointPositions);
}


brics_actuator::JointPositions createArmPositionMsg(JointValues jointAngles)
{
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

bool YoubotManipulator::goToPose(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res)
{
    Pose pose;
    pose.position(0) = req.pose.position[0];
    pose.position(1) = req.pose.position[1];
    pose.position(2) = req.pose.position[2];
    pose.orientation(0) = req.pose.orientation[0];
    pose.orientation(1) = req.pose.orientation[1];
    pose.orientation(2) = req.pose.orientation[2];

    moveArm(pose);
    res.feasible = true;
    return true;
}

bool YoubotManipulator::graspObject(arm_kinematics::ManipulatorPose::Request & req, arm_kinematics::ManipulatorPose::Response & res)
{
    JointValues jointAngles;
    // TODO add dynamic object height
    double objectHeight = 0.05;
    res.feasible = false;

    // First Step --------
    Pose startPose;
    startPose.position(0) = req.pose.position[0];
    startPose.position(1) = req.pose.position[1];
    startPose.position(2) = req.pose.position[2];
    startPose.orientation(0) = req.pose.orientation[0];
    startPose.orientation(1) = req.pose.orientation[1];
    startPose.orientation(2) = req.pose.orientation[2];
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
    ros::Duration(2).sleep();

    moveToLineTrajectory(startPose, endPose);
    moveGripper(0.0);
    ros::Duration(2).sleep();
   
    res.feasible = true;
    return true;
}

void YoubotManipulator::moveToLineTrajectory(const Pose & startPose, const Pose & endPose)
{
    JointValues startAngles;
    brics_actuator::JointPositions jointPositions;
    double startVel = 0;
    double endVel = 0;

    ROS_INFO_STREAM("[Arm Manipulation] Max Vel: " << maxVel << "\t Max Accel: " << maxAccel);
    TrajectoryGenerator gen(maxVel, maxAccel, 1/lr);
    gen.calculateTrajectory(startPose, endPose);

    ROS_INFO_STREAM("[Arm Manipulation] Move to initial position.");
    for (size_t i = 0; i < 5; ++i) {
        startAngles(i) = gen.trajectory.points[0].positions[i];
    }
    jointPositions = createArmPositionMsg(startAngles);
    armPublisher.publish(jointPositions);
    ros::Duration(2).sleep();


    if (!gen.trajectory.points.empty())
    {
        trajectoryAC->waitForServer(); // Will wait for infinite time
        ROS_INFO("[Arm Manipulation] Action server started, sending goal.");

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = gen.trajectory;

        trajectoryAC->sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = trajectoryAC->waitForResult(ros::Duration(60.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = trajectoryAC->getState();
            ROS_INFO("[Arm Manipulation] Action finished: %s", state.toString().c_str());
        } else ROS_ERROR("[Arm Manipulation] Action did not finish before the time out.");
    }
}

void YoubotManipulator::moveArmLoop()
{
    ROS_INFO_STREAM("[Arm Manipulation] Load all modules.");
    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /grasp_object...");
    trajectoryServer = nh.advertiseService("grasp_object", &YoubotManipulator::graspObject, this);

    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /manipulator_pose...");
    poseServer = nh.advertiseService("manipulator_pose", &YoubotManipulator::goToPose, this);

    ROS_INFO_STREAM("[Arm Manipulation] Load ActionClient arm_1/arm_controller/velocity_joint_trajecotry");
    trajectoryAC = new ActionClent("arm_1/arm_controller/velocity_joint_trajecotry", true);

    ROS_INFO_STREAM("[Arm Manipulation] Params:"
        << " lr: " << lr << "\t"
        << " max_vel: " << maxVel << "\t"
        << " max_accel: " << maxAccel);
    
    while(nh.ok()) {
        ros::spin();
    }
}
