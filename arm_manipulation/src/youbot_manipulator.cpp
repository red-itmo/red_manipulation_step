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
    std::vector<JointValues> solutions;

    ROS_INFO_STREAM("[Arm Manipulation] Position: (" << pose.position(0) << " " << pose.position(1) << " " << pose.position(2) << ")");
    ROS_INFO_STREAM("[Arm Manipulation] Orientation: (" << pose.orientation(0) << " " << pose.orientation(1) << " " << pose.orientation(2) << ")");
    
    if (solver.solveIK(pose, solutions)) {
        JointValues jointAngles = solutions[0];
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
bool YoubotManipulator::followTrajectory(arm_kinematics::PoseArray::Request & req, arm_kinematics::PoseArray::Response & res)
{
    JointValues angles;
    std::vector<JointValues> sol;
    double objectHeight = 0.05, alpha = 0;
    res.feasible = false;

    // First Step --------
    Pose startPose;
    startPose.position(0) = req.pose_array[0].position[0];
    startPose.position(1) = req.pose_array[0].position[1];
    startPose.position(2) = req.pose_array[0].position[2];
    startPose.orientation(0) = req.pose_array[0].orientation[0];
    startPose.orientation(1) = req.pose_array[0].orientation[1];
    startPose.orientation(2) = req.pose_array[0].orientation[2];
    Pose endPose = startPose;

    startPose.position(2) += objectHeight;
    if (!solver.solveIK(startPose, sol)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPose.position(0) << ", " << startPose.position(1)  << ", " << startPose.position(2));
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    startPose.orientation(2) = alpha;

    endPose.orientation(2) = alpha; // ???????????????
    if (!solver.solveIK(endPose, sol)) {
        ROS_FATAL_STREAM("Solution is not found (endPos): " << endPose.position(0) << ", " << endPose.position(1)  << ", " << endPose.position(2));
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    endPose.orientation(2) = alpha;
    
    moveArm(startPose);
    ros::Duration(1).sleep();
    moveGripper(0.0115);
    ros::Duration(2).sleep();

    moveToLineTrajectory(startPose, endPose);
    moveGripper(0.0);
    ros::Duration(2).sleep();

    // Second Step --------

    startPose.position(0) = req.pose_array[1].position[0];
    startPose.position(1) = req.pose_array[1].position[1];
    startPose.position(2) = req.pose_array[1].position[2];
    startPose.orientation(0) = req.pose_array[1].orientation[0];
    startPose.orientation(1) = req.pose_array[1].orientation[1];
    startPose.orientation(2) = req.pose_array[1].orientation[2];
    endPose = startPose;

    startPose.position(2) += 0.05;
    if (!solver.solveIK(startPose, sol)) {
        ROS_FATAL_STREAM("Solution is not found (startPos): " << startPose.position(0) << ", " << startPose.position(1)  << ", " << startPose.position(2));
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    startPose.orientation(2) = alpha;

    endPose.orientation(2) = alpha;
    if (!solver.solveIK(endPose, sol)) {
        ROS_FATAL_STREAM("Solution is not found (endPos): " << endPose.position(0) << ", " << endPose.position(1)  << ", " << endPose.position(2));
        return 1;
    }
    angles = sol[0];
    alpha = angles(1) + angles(2) + angles(3);
    endPose.orientation(2) = alpha;

    moveArm(startPose);
    ros::Duration(2).sleep();

    moveToLineTrajectory(startPose, endPose);
    moveGripper(0.0115);
    ros::Duration(2).sleep();

    res.feasible = true;
    return true;
}

void YoubotManipulator::moveToLineTrajectory(const Pose & startPose, const Pose & endPose)
{
    std::vector<JointValues> sol;
    double startVel = 0;
    double endVel = 0;

    ROS_INFO_STREAM("[Arm Manipulation] Max Vel: " << maxVel << "\t Max Accel: " << maxAccel);
    TrajectoryGenerator gen(maxVel, maxAccel, 1/lr);
    gen.calculateTrajectory(startPose, endPose);

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
    ROS_INFO_STREAM("[Arm Manipulation] Service [server]: /object_position...");
    trajectoryServer = nh.advertiseService("grasp_poses", &YoubotManipulator::followTrajectory, this);

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
