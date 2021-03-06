#include <arm_kinematics/ArmKinematics.h>
#include <arm_kinematics/KinematicConstants.h>
#define KINEMATIC_DEBUG false

ArmKinematics::ArmKinematics()
{}

ArmKinematics::~ArmKinematics()
{}

bool ArmKinematics::solveSpaceIK(Vector3d & position, matrix::SquareMatrix<double, 3> & orientation, JointValues & jointAngles, double & alpha)
{
    double dx = 0, dz = 0;

    dx = position(0) - d0x;
    dz = position(2) - d0z;

    jointAngles(0) = atan2(position(1), dx);
    if (jointAngles(0) > M_PI / 2) jointAngles(0) -= M_PI;
    else if (jointAngles(0) < -M_PI / 2) jointAngles(0) += M_PI;

    alpha = atan2(orientation(0, 2) * cos(jointAngles(0)) + orientation(1, 2) * sin(jointAngles(0)), orientation(2, 2));
    jointAngles(4) = atan2(
                         -orientation(0, 0) * sin(jointAngles(0)) + orientation(1, 0) * cos(jointAngles(0)),
                         cos(alpha) * (sin(jointAngles(0)) * orientation(1, 0) + cos(jointAngles(0)) * orientation(0, 0)) - sin(alpha) * orientation(2, 0)
                     );

    // Parallel shift and rotation of goal position
    dx = cos(jointAngles(0)) * dx + sin(jointAngles(0)) * position(1) - d1x;
    dz -= d1z;

    position(0) = dx;
    position(2) = dz;

    return true;
}

bool ArmKinematics::solveSpaceIK(Vector3d & position, const Vector3d & orientation, JointValues & jointAngles, double & alpha)
{
    matrix::SquareMatrix<double, 3> orientationMatrix =
        calcOrientationMatrix(orientation(0), orientation(1), orientation(2));
    return solveSpaceIK(position, orientationMatrix, jointAngles, alpha);
}

bool ArmKinematics::solvePlaneIK(Vector3d position, const double alpha, JointValues & jointAngles, const int configuration)
{
    double cosPhi3;
    bool valid = true;

    position(0) -= (d4 + griperLength) * sin(alpha);
    position(2) -= (d4 + griperLength) * cos(alpha);

    // Check length of setting goal
    if (sqrt(position(0)*position(0) + position(2)*position(2)) > d2 + d3) {
        // ROS_FATAL_STREAM("Solution NOT exists!!");
        return false;
    }

    cosPhi3 = (position(0) * position(0) + position(2) * position(2) - d2 * d2 - d3 * d3) / (2 * d2 * d3);
    if (cosPhi3 > 1) jointAngles(2) = 0;
    else if (cosPhi3 < -1) jointAngles(2) = M_PI;
    else jointAngles(2) = configuration * atan2(sqrt(1 - cosPhi3 * cosPhi3), cosPhi3);
    // else jointAngles(2) = offset[i]*atan2(sqrt(1 - cosPhi3*cosPhi3), cosPhi3);

    jointAngles(1) = atan2(position(0), position(2)) - atan2(d3 * sin(jointAngles(2)), d2 + d3 * cos(jointAngles(2)));
    jointAngles(3) = alpha - jointAngles(2) - jointAngles(1);

    // Constraints checking
    for (uint j = 1; j < 4; ++j) {
        valid = valid && (jointMinAngles[j] <= jointAngles(j) && jointAngles(j) <= jointMaxAngles[j]);
    }

    return valid;
}

bool ArmKinematics::solveIK(const Vector3d & position, matrix::SquareMatrix<double, 3> & orientation, JointValues & jointAngles, const int configuration)
{
    Vector3d goalPosition = position;
    double alpha = 0; // summ q2 - q5; q - joint angle
    double startAlpha;
    bool sign = false; // for switch sign of alpha

    if (configuration != -1 && configuration != 1) {
        ROS_FATAL_STREAM("Configuration: " << configuration << " is not valid.");
        return false;
    }

    solveSpaceIK(goalPosition, orientation, jointAngles, alpha);

    startAlpha = alpha;
    uint i = 0;

    // Solve IK in plane space
    while (alpha > startAlpha - M_PI / 2 && alpha < startAlpha + M_PI / 2) {

        alpha = startAlpha + 0.01 * (-1 + 2 * sign) * i;
        sign = !sign;
        i++;

        if (solvePlaneIK(goalPosition, alpha, jointAngles, configuration)) return true;

    }
    return false;
}

matrix::SquareMatrix<double, 3> ArmKinematics::calcOrientationMatrix(double phi1, double phi5, double alpha)
{
    matrix::SquareMatrix<double, 3> orientation;
    orientation(0, 0) = cos(phi1) * cos(alpha) * cos(phi5) - sin(phi1) * sin(phi5);
    orientation(0, 1) = -cos(phi1) * cos(alpha) * sin(phi5) - sin(phi1) * cos(phi5);
    orientation(0, 2) = cos(phi1) * sin(alpha);

    orientation(1, 0) = sin(phi1) * cos(alpha) * cos(phi5) - cos(phi1) * sin(phi5);
    orientation(1, 1) = -sin(phi1) * cos(alpha) * sin(phi5) + cos(phi1) * cos(phi5);
    orientation(1, 2) = sin(phi1) * sin(alpha);

    orientation(2, 0) = -sin(alpha) * cos(phi5);
    orientation(2, 1) = sin(alpha) * sin(phi5);
    orientation(2, 2) = cos(alpha);

    return orientation;
}

bool ArmKinematics::solveIK(const Vector3d & position, const double phi5, const double alpha, JointValues & jointAngles, const int configuration)
{

    double phi1 = atan2(position(1), (position(0) - d0x));
    if (phi1 < jointMaxAngles[0] || phi1 > jointMinAngles[0]) phi1 += M_PI;
    matrix::SquareMatrix<double, 3> orientation = calcOrientationMatrix(phi1, phi5, alpha);

    return ArmKinematics::solveIK(position, orientation, jointAngles, configuration);
}
bool ArmKinematics::solveIK(const Vector3d & position, const Vector3d & orientation, JointValues & jointAngles, const int configuration)
{
    matrix::SquareMatrix<double, 3> orientationMatrix = calcOrientationMatrix(orientation(0), orientation(1), orientation(2));
    return ArmKinematics::solveIK(position, orientationMatrix, jointAngles, configuration);
}
bool ArmKinematics::solveIK(const Pose & position, JointValues & jointAngles, const int configuration)
{
    matrix::SquareMatrix<double, 3> orientation =
        calcOrientationMatrix(position.orientation(0), position.orientation(1), position.orientation(2));

    return ArmKinematics::solveIK(position.position, orientation, jointAngles, configuration);
}

bool ArmKinematics::solveFullyIK(const Pose & position, JointValues & jointAngles)
{
    matrix::SquareMatrix<double, 3> orientation =
        calcOrientationMatrix(position.orientation(0), position.orientation(1), position.orientation(2));

    if (solveIK(position.position, orientation, jointAngles, 1)) return true;
    else return solveIK(position.position, orientation, jointAngles, -1);
}

void ArmKinematics::calcKinematicsParams(const Vector3d & linearVelocities, const JointValues & jointAngles, JointValues & jointAngVel, JointValues & jointAngAcc)
{
    double dJ[2][2];
    double invJ[2][2];

    // Inverse Jacobian
    invJ[0][0] = sin(jointAngles(2) + jointAngles(1)) / (d2 * sin(jointAngles(2)));
    invJ[0][1] = cos(jointAngles(2) + jointAngles(1)) / (d2 * sin(jointAngles(2)));
    invJ[1][0] = -(d2 * sin(jointAngles(1)) + d3 * sin(jointAngles(2) + jointAngles(1))) / (d2 * d3 * sin(jointAngles(2)));
    invJ[1][1] = -(d2 * cos(jointAngles(1)) + d3 * cos(jointAngles(2) + jointAngles(1))) / (d2 * d3 * sin(jointAngles(2)));

    // Calculate angular velocity of joint
    jointAngVel(1) = invJ[0][0] * linearVelocities(0) + invJ[0][1] * linearVelocities(2);
    jointAngVel(2) = invJ[1][0] * linearVelocities(0) + invJ[1][1] * linearVelocities(2);
    jointAngVel(3) = - jointAngVel(1) - jointAngVel(2);

    // Check joint velocities
    // for (uint i = 1; i <= 3; ++i) {
    //     if (jointAngVel(i) < jointMinVel[i] || jointAngVel(i) > jointMaxVel[i])
    //         jointAngVel(i) = (jointAngVel(i) > 0) - (jointAngVel(i) < 0);
    // }

    // Jacobian derivative
    dJ[0][0] = -d3 * (jointAngVel(1) + jointAngVel(2)) * sin(jointAngles(1) + jointAngles(2)) - d2 * jointAngVel(1) * sin(jointAngles(1));
    dJ[0][1] = -d3 * (jointAngVel(1) + jointAngVel(2)) * sin(jointAngles(1) + jointAngles(2));
    dJ[1][0] = -d3 * (jointAngVel(1) + jointAngVel(2)) * cos(jointAngles(1) + jointAngles(2)) - d2 * jointAngVel(1) * cos(jointAngles(1));
    dJ[1][1] = -d3 * (jointAngVel(1) + jointAngVel(2)) * cos(jointAngles(1) + jointAngles(2));

    // Calculate acceleration of joint
    jointAngAcc(1) = -invJ[0][0] * (dJ[0][0] * jointAngVel(1) + dJ[0][1] * jointAngVel(2)) - invJ[0][1] * (dJ[1][0] * jointAngVel(1) + dJ[1][1] * jointAngVel(2));
    jointAngAcc(2) = -invJ[1][0] * (dJ[0][0] * jointAngVel(1) + dJ[0][1] * jointAngVel(2)) - invJ[1][1] * (dJ[1][0] * jointAngVel(1) + dJ[1][1] * jointAngVel(2));
    jointAngAcc(3) = -jointAngAcc(1) - jointAngAcc(2);

    // ROS_INFO_STREAM("-- lin vel: " << linearVelocities(0) << ", " << linearVelocities(1) << ", " << linearVelocities(2));

    // ROS_INFO_STREAM("vel: ("
    //     << jointAngVel(0) << ", "
    //     << jointAngVel(1) << ", "
    //     << jointAngVel(2) << ", "
    //     << jointAngVel(3) << ", "
    //     << jointAngVel(4) << ")");
    // ROS_INFO_STREAM("accel: ("
    //     << jointAngAcc(0) << ", "
    //     << jointAngAcc(1) << ", "
    //     << jointAngAcc(2) << ", "
    //     << jointAngAcc(3) << ", "
    //     << jointAngAcc(4) << ")");

    return;
}

Vector3d ArmKinematics::transformFromFrame5ToFrame0(const JointValues & jointAngles, const Vector3d & position)
{

    Vector3d resultPosition;
    double phi1 = jointAngles(0);
    double alpha = jointAngles(1) + jointAngles(2) + jointAngles(3);
    double phi5 = jointAngles(4);

    // xz-plane shift
    double dx = d2 * sin(jointAngles(1)) + d3 * sin(jointAngles(1) + jointAngles(2)) + d4 * sin(alpha) + d1x;
    double dz = d2 * cos(jointAngles(1)) + d3 * cos(jointAngles(1) + jointAngles(2)) + d4 * cos(alpha) + d1z;

    // x-coordinate
    resultPosition(0) = (cos(alpha) * cos(phi1) * cos(phi5) - sin(phi1) * sin(phi5)) * position(0)
                        - (cos(alpha) * cos(phi1) * sin(phi5) + sin(phi1) * cos(phi5)) * position(1)
                        + (sin(alpha) * cos(phi1)) * position(2)
                        + dx * cos(phi1) + d0x;
    // y-coordinate
    resultPosition(1) = (cos(phi1) * sin(phi5) + cos(alpha) * sin(phi1) * cos(phi5)) * position(0)
                        + (cos(phi1) * cos(phi5) - cos(alpha) * sin(phi1) * sin(phi5)) * position(1)
                        + (sin(alpha) * sin(phi1)) * position(2)
                        + dx * sin(phi1);
    // z-coordinate
    resultPosition(2) =  - (sin(alpha) * cos(phi5)) * position(0)
                         + (sin(alpha) * sin(phi5)) * position(1)
                         + (cos(alpha)) * position(2)
                         + dz + d0z;

    return resultPosition;
}

void makeYoubotArmOffsets(JointValues & jointAngles)
{
    jointAngles(0) = jointOffsets[0] - jointAngles(0);
    jointAngles(1) += jointOffsets[1];
    jointAngles(2) += jointOffsets[2];
    jointAngles(3) += jointOffsets[3];
    jointAngles(4) = jointOffsets[4] - jointAngles(4);
}

void makeKinematicModelOffsets(JointValues & jointAngles)
{
    jointAngles(0) = jointOffsets[0] - jointAngles(0);
    jointAngles(1) -= jointOffsets[1];
    jointAngles(2) -= jointOffsets[2];
    jointAngles(3) -= jointOffsets[3];
    jointAngles(4) = jointOffsets[4] - jointAngles(4);
}

bool checkAngles(const JointValues & jointAngles)
{
    for (size_t i = 0; i < DOF; ++i) {
        if (jointMinAngles[i] > jointAngles(i) || jointMaxAngles[i] < jointAngles(i))
        {
            ROS_WARN("[ArmKinematics] Joint %zu is out of range(%f)", i, jointAngles(i));
            return false;
        }
    }
    return true;
}

Vector3d ArmKinematics::calcMaxRot(const Vector3d & position)
{
    Vector3d jointValues;
    double d23 = d2 + d3;
    double d = d4 + griperLength;
    Vector3d goal;
    goal = position - Vector3d(d0x, 0, d0z);
    int sgn = sign(goal(0));
    double q1;
    q1 = atan2(goal(1), sgn * goal(0));   //TODO: check
    // Shift and rotate coordinate system
    goal(0) = goal(0) * cos(q1) + goal(1) * sin(q1) - d1x;
    goal(1) = -goal(0) * sin(q1) + goal(1) * cos(q1);
    goal(2) = goal(2) - d1z;
    bool triangleExists = (goal.norm() < d23 + d) && (d23 < goal.norm() + d) && (d < goal.norm() + d23);
    if (d23 + d < goal.norm() || !triangleExists)
    {
        ROS_WARN("[ArmKinematics] Solution doesn't exist!");
        jointValues.setAll(-1000);
        return jointValues;
    }
    double cosq4 = (pow(goal.norm(), 2) - d23 * d23 - d * d) / (2 * d23 * d);
    jointValues(2) = sgn * atan2(sqrt(1 - cosq4 * cosq4), cosq4);

    jointValues(0) = atan2(goal(0), goal(2)) - atan2(d * sin(jointValues(2)), d23 + d * cos(jointValues(2)));

    jointValues(1) = 0;
    Vector3d d34Vec;
    double d34, cosq3;
    if (jointValues(2) > jointMaxAngles[3])
    {
        jointValues(2) = jointMaxAngles[3];
        d34Vec(0) = d * sin(jointValues(2));
        d34Vec(1) = 0;
        d34Vec(2) = d3 + d * cos(jointValues(2));
        d34 = d34Vec.norm();
        cosq3 = (pow(goal.norm(), 2) - d2 * d2 - d34 * d34) / (2 * d2 * d34);
        jointValues(1) = atan2(sqrt(1 - cosq3 * cosq3), cosq3);
        if(std::isnan(jointValues(1)))
            ROS_FATAL_STREAM("cosq3 is negative!!");
        // std::cout<<"jv1:"<<jointValues(1)<<"\n";

        jointValues(0) = atan2(goal(0), goal(2)) - atan2(d34 * sin(jointValues(1)), d2 + d34 * cos(jointValues(1)));
        jointValues(1) = jointValues(1) - atan2(d * sin(jointValues(2)), d3 + d * cos(jointValues(2)));

        if (KINEMATIC_DEBUG)
            ROS_INFO("jointValues(2) > jointMaxAngles[3]");
    }

    if (jointValues(2) < jointMinAngles[3])
    {
        jointValues(2) = jointMinAngles[3];
        d34Vec(0) = d * sin(jointValues(2));
        d34Vec(1) = 0;
        d34Vec(2) = d3 + d * cos(jointValues(2));
        d34 = d34Vec.norm();
        cosq3 = (pow(goal.norm(), 2) - d2 * d2 - d34 * d34) / (2 * d2 * d34);
        jointValues(1) = atan2(sqrt(1 - cosq3 * cosq3), cosq3);
        if(std::isnan(jointValues(1)))
            ROS_FATAL_STREAM("cosq3 is negative!!");

        jointValues(0) = atan2(goal(0), goal(2)) - atan2(d34 * sin(jointValues(1)), d2 + d34 * cos(jointValues(1)));
        jointValues(1) = jointValues(1) - atan2(d * sin(jointValues(2)), d3 + d * cos(jointValues(2)));

        if (KINEMATIC_DEBUG)
            ROS_INFO("jointValues(2) < jointMaxAngles[3]");
    }

    if (jointValues(0) > jointMaxAngles[1])
    {
        jointValues(0) = jointMaxAngles[1];
        d34Vec(0) = goal(0) - d2 * sin(jointValues(0));
        d34Vec(1) = goal(1);
        d34Vec(2) = goal(2) - d2 * cos(jointValues(0));
        cosq4 = (pow(d34Vec.norm(), 2) - d3 * d3 - d * d) / (2 * d3 * d);
        jointValues(2) = sgn * atan2(sqrt(1 - cosq4 * cosq4), cosq4);
        if(std::isnan(jointValues(2)))
            ROS_FATAL_STREAM("cosq4 is negative!!");

        double q23 = atan2(d34Vec(0), d34Vec(2)) - atan2(d*sin(jointValues(2)), d3 + d*cos(jointValues(2)));
        jointValues(1) = q23 - jointValues(0);

        if (KINEMATIC_DEBUG)
            ROS_INFO("jointValues(0) > jointMaxAngles[1]");
    }

    if (jointValues(0) < jointMinAngles[1])
    {
        jointValues(0) = jointMinAngles[1];
        d34Vec(0) = goal(0) - d2 * sin(jointValues(0));
        d34Vec(1) = goal(1);
        d34Vec(2) = goal(2) - d2 * cos(jointValues(0));
        cosq4 = (pow(d34Vec.norm(), 2) - d3 * d3 - d * d) / (2 * d3 * d);
        jointValues(2) = sgn * atan2(sqrt(1 - cosq4 * cosq4), cosq4);
        if(std::isnan(jointValues(2)))
            ROS_FATAL_STREAM("cosq4 is negative!!");

        double q23 = atan2(d34Vec(0), d34Vec(2)) - atan2(d*sin(jointValues(2)), d3 + d*cos(jointValues(2)));
        jointValues(1) = q23 - jointValues(0);

        if (KINEMATIC_DEBUG)
            ROS_INFO("jointValues(0) < jointMaxAngles[1]");
    }


    if (KINEMATIC_DEBUG) {
        std::cout << "goal: \n" << goal << std::endl;
        std::cout << "d34: \n" << d34Vec << std::endl;
    }

    return jointValues;
}

Vector3d ArmKinematics::ForwardKin(const JointValues &angles)
{
    Vector3d zeros;

    return transformFromFrame5ToFrame0(angles, zeros);
}
Vector3d ArmKinematics::ForwardKinWithGripper(const JointValues &angles)
{
    Vector3d vec(0, 0, griperLength);

    return transformFromFrame5ToFrame0(angles, vec);
}

matrix::SquareMatrix<double, 5> ArmKinematics::Jacobian(const JointValues &angles)
{
    matrix::SquareMatrix<double, 5> j = matrix::zeros<double, 5, 5>();
    Vector3d d;
    d(0) = 0;
    d(1) = 0;
    d(2) = d4 + griperLength;

    j(0, 0) = -sin(angles(0)) * (d1x + d2 * sin(angles(1)) + d3 * sin(angles(2) + angles(1)) + d(2) * sin(angles(3) + angles(2) + angles(1)));
    j(1, 0) = cos(angles(0)) * (d1x + d2 * sin(angles(1)) + d3 * sin(angles(2) + angles(1)) + d(2) * sin(angles(3) + angles(2) + angles(1)));
    j(2, 0) = 0;
    j(3, 0) = 0;
    j(4, 0) = 0;

    j(0, 1) = cos(angles(0)) * (d(2) * cos(angles(3) + angles(2) + angles(1)) + d3 * cos(angles(2) + angles(1)) + d2 * cos(angles(1)));
    j(1, 1) = sin(angles(0)) * (d2 * cos(angles(1)) + d3 * cos(angles(2) + angles(1)) + d(2) * cos(angles(3) + angles(2) + angles(1)));
    j(2, 1) = -d(2) * sin(angles(3) + angles(2) + angles(1)) - d3 * sin(angles(2) + angles(1)) - d2 * sin(angles(1));
    j(3, 1) = 1;
    j(4, 1) = 0;

    j(0, 2) = cos(angles(0)) * (d(2) * cos(angles(3) + angles(2) + angles(1)) + d3 * cos(angles(2) + angles(1)));
    j(1, 2) = sin(angles(0)) * (d3 * cos(angles(2) + angles(1)) + d(2) * cos(angles(3) + angles(2) + angles(1)));
    j(2, 2) = -d(2) * sin(angles(3) + angles(2) + angles(1)) - d3 * sin(angles(2) + angles(1));
    j(3, 2) = 1;
    j(4, 2) = 0;

    j(0, 3) = d(2) * cos(angles(0)) * cos(angles(3) + angles(2) + angles(1));
    j(1, 3) = d(2) * sin(angles(0)) * cos(angles(3) + angles(2) + angles(1));
    j(2, 3) = -d(2) * sin(angles(3) + angles(2) + angles(1));
    j(3, 3) = 1;
    j(4, 3) = 0;

    j(0, 4) = 0;
    j(1, 4) = 0;
    j(2, 4) = 0;
    j(3, 4) = 0;
    j(4, 4) = 1;
    return j;
}


JointValues ArmKinematics::numericalIK(const Pose &pose, JointValues q)
{
    int iter_num = 1000, iter = 0;
    matrix::SquareMatrix<double, 5> j, temp;
    double k = 0.05;

    // Actally joint values are not stored here
    //Used just for calculations
    JointValues matrixFK, poseAndRot, error, dq;

    // Fill vector for IK
    for (int i = 0; i < 3; ++i) {
        poseAndRot(i) = pose.position(i);
    }
    poseAndRot(3) = pose.orientation(0);
    poseAndRot(4) = pose.orientation(1);

    // Solve FK
    Vector3d FKoutput = ForwardKinWithGripper(q);
    matrixFK(0) = FKoutput(0);
    matrixFK(1) = FKoutput(1);
    matrixFK(2) = FKoutput(2);
    matrixFK(3) = q(1) + q(2) + q(3);
    matrixFK(4) = q(4);

    // Find error between desired vector and real
    error = poseAndRot - matrixFK;
    while (error.norm() > 10e-10 && iter < iter_num)
    {
        j = Jacobian(q);
        temp = j.transpose()*j + k*k*matrix::eye<double,5>();
        temp= temp.I();
        dq = temp*j.transpose() * error;
        q += dq;

        FKoutput = ForwardKinWithGripper(q);
        matrixFK(0) = FKoutput(0);
        matrixFK(1) = FKoutput(1);
        matrixFK(2) = FKoutput(2);
        matrixFK(3) = q(1) + q(2) + q(3);
        matrixFK(4) = q(4);
        error = poseAndRot - matrixFK;
        iter++;
    }
    prevNumIKAngle = q;
    // std::cout<<"total iterations:" << iter<<std::endl;
    if (iter > iter_num - 1)
    {
        //solution not found
        q.setAll(-1000);
        return q;
    }
    if (!checkAngles(q))
    {
        //solution not found
        q.setAll(-2000);
        return q;
    }
    for (int i = 0; i < 5; ++i)
        q(i) = round(q(i) * 10000) / 10000;
    return q;
}