#include <trajectory_generator/TrajectoryGenerator.h>

Trajectory::Trajectory()
{}

Trajectory::~Trajectory()
{}

void Trajectory::exponencialMovingAvarage(std::vector<JointValues> & data, std::vector<JointValues> & movingAvarage, const double alpha)
{
	size_t n = data.size();
	size_t nAvg = movingAvarage.size();
	if (n != nAvg)
		movingAvarage.resize(n);

	movingAvarage[0] = data[0];
	for (size_t i = 1; i < n; ++i) {
		movingAvarage[i] = alpha * data[i] + (1 - alpha) * movingAvarage[i - 1];
	}
}

double Trajectory::getVel(double time)
{
	// ROS_INFO_STREAM("directionSign:"<< directionSign<<" mAc:"<<maxAccel<<" t:"<<time<<" v:"<<maxVel);
	if (time >= 0 && time < t1)
		return directionSign * maxAccel * time;
	if (time >= t1 && time < t2)
		return directionSign * maxVel;
	if (time >= t2 && time <= t3)
		return directionSign * (maxVel - maxAccel * (time - t2));
	// ROS_WARN_STREAM("[Trajectory]time>t3");
	return 0;
}

Vector3d Trajectory::getRotVel(double time)
{
	if (time >= 0 && time <= t3)
		return rotVel;
	rotVel(0) = 0;
	rotVel(1) = 0;
	rotVel(2) = 0;
	return rotVel;
}
void Trajectory::calculateWorkSpaceTrajectory(const double maxVel, const double maxAccel, const Pose & startPose, const Pose & endPose, const double timeStep)
{
	this->maxAccel = maxAccel;
	this->maxVel = maxVel;
	Vector3d positionDiff = endPose.position - startPose.position;
	Vector3d movementDirection = positionDiff;
	movementDirection.normalize();
	directionSign = std::abs(positionDiff.norm()) / positionDiff.norm();

	ROS_DEBUG_STREAM("[trajectory] Max Vel: " << maxVel << " Max Accel: " << maxAccel << " TimeStep: " << timeStep);
	// Calculate trajectory times
	t1 = maxVel / maxAccel;
	t2 = positionDiff.norm() / maxVel;
	t3 = t2 + t1;

	ROS_INFO_STREAM("[Trajectory] Time variables (" << t1 << ", " << t2 << ", " << t3 << ")");

	// rotation trajectory
	double theta_i = startPose.orientation(0),
	       theta_e = endPose.orientation(0),
	       psi_i = startPose.orientation(1),
	       psi_e = endPose.orientation(1);
	rotVel(0) = (theta_e - theta_i) / t3;
	rotVel(1) = (psi_e - psi_i) / t3;

	if (t2 < t1)
	{
		ROS_FATAL_STREAM("[Trajectory]t2<t1");
		return;
	}
	Vector3d currCoord = startPose.position;
	Vector3d currRot(theta_i, psi_i, 0);
	Vector3d currVel;

	for (double currentTime = 0; currentTime < t3; currentTime += timeStep)
	{
		currVel = movementDirection * getVel(currentTime);
		velTra.push_back(currVel);
		posTra.push_back(currCoord);
		time.push_back(currentTime);
		rotTra.push_back(currRot);
		currCoord += timeStep / 2 * (currVel + movementDirection * getVel(currentTime + timeStep));
		currRot += timeStep / 2 * (getRotVel(currentTime) + getRotVel(currentTime + timeStep));
	}
	ROS_INFO_STREAM("[trajectory]trj is generated, number of steps:"<<rotTra.size());
}

void Trajectory::convertWorkSpaceToJointSpace(Pose startPose, Pose endPose, const double timeStep)
{
	JointValues currJntAng, currJntAngVel, currJntAngAcc;
	JointValues prevAngles, currAngles, nextAngles, anglesDiff, zeroVec;
	zeroVec.setZero();
	ArmKinematics solver;
	double prevTime, currTime, nextTime;

	Vector3d rotVel, startRot, endRot, currRot, currPosition;
	double startAlpha, endAlpha, alpha, alphaVel;

	startRot = startPose.orientation;
	endRot = endPose.orientation;

	std::vector<Pose> poses;
	Pose pose;
	for (int i = 0; i < posTra.size(); ++i) {
		pose.position = posTra[i];
		pose.orientation(0) = rotTra[i](0);
		pose.orientation(1) = rotTra[i](1);
		pose.orientation(2) = rotTra[i](2);
		poses.push_back(pose);
	}
	Vector3d angles1 = solver.calcMaxRot(poses.front().position);
	Vector3d angles2 = solver.calcMaxRot(poses.back().position);
	double theta1 = angles1(0) + angles1(1) + angles1(2);
	double theta2 = angles2(0) + angles2(1) + angles2(2);
	if (theta1 > M_PI)
		theta1 = M_PI;
	if (theta2 > M_PI)
		theta2 = M_PI;

	double err = 0, offset = 0, theta;
	Pose curConf;
	Vector3d ang;
	JointValues curRot, maxRot = matrix::zeros<double, 5, 1>();
	for (size_t i = 0; i < poses.size(); ++i)
	{
		curConf = poses[i];
		ang = solver.calcMaxRot(curConf.position);


		maxRot(1) = M_PI / 3;
		maxRot(2) = M_PI / 6;
		maxRot(3) = M_PI / 6;
		theta = ang(0) + ang(1) + ang(2);
		offset = 0;
		if (theta > M_PI - 0.1)
			offset = pow(theta + 0.1 - M_PI, 2);
		else
			offset = 0;
		curConf.orientation(0) = theta - 0.1 - offset;
		curRot = solver.numericalIK(curConf, maxRot);
		if(curRot(0)==-1000)
			return;
		qTra.push_back(curRot);
	}
}

void Trajectory::generateTrajectoryMsg(trajectory_msgs::JointTrajectory & trajectory)
{
	ROS_INFO("Generating message...");
	JointValues currJntAng, currJntAngAcc, currJntAngVel;
	double timeStep = time[1] - time[0];

	for (size_t p = 0; p < qTra.size(); ++p) {

		trajectory_msgs::JointTrajectoryPoint point;

		currJntAng = qTra[p];
		currJntAngAcc = qdotTra[p];
		currJntAngVel = qdotdotTra[p];

		for (size_t i = 0; i < 5; ++i) {
			point.positions.push_back(currJntAng(i));
			// ROS_DEBUG_ONCE("[trajectory]Publishing trajectory angles:");
			// ROS_DEBUG("[trajectory]%f",currJntAng(i));
			point.velocities.push_back(currJntAngAcc(i));
			point.accelerations.push_back(currJntAngVel(i));
			point.time_from_start = ros::Duration(p * timeStep);
		}
		trajectory.points.push_back(point);
	}

	if (trajectory.joint_names.size() == 5) return;
	else trajectory.joint_names.resize(5);

	for (size_t i = 0; i < 5; ++i) {
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		trajectory.joint_names[i] = jointName.str();
	}
}