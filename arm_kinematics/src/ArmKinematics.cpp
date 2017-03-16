#include <arm_kinematics/ArmKinematics.h>
#include <arm_kinematics/KinematicConstants.h>

ArmKinematics::ArmKinematics() 
{}

ArmKinematics::~ArmKinematics()
{}

bool ArmKinematics::solveIK(const Vector3d & position, matrix::Matrix<double, 3, 3> & orientation, std::vector<JointValues> & solutions)
{
    bool valid = true;
    double alpha = 0; // summ of phi(2) - phi(5), parameter
    double dx = 0, dz = 0, px, pz;
    // double offset[2] = {1, -1}; 
    double cosPhi3 = 0;
    JointValues phi;
    bool sign = false; 
    double startAlpha;

    solutions.clear();

    dx = position(0) - d0x;
    dz = position(2) - d0z;

    // Search the angles phi(1) - phi(5)
    phi(0) = atan2(position(1), dx);
    if (phi(0) > M_PI/2) phi(0) -= M_PI;
    else if (phi(0) < -M_PI/2) phi(0) += M_PI;

    alpha = atan2(orientation(0, 2)*cos(phi(0)) + orientation(1, 2)*sin(phi(0)), orientation(2, 2));
    phi(4) = atan2(
        -orientation(0, 0)*sin(phi(0)) + orientation(1, 0)*cos(phi(0)),
        cos(alpha)*(sin(phi(0))*orientation(1, 0) + cos(phi(0))*orientation(0, 0)) - sin(alpha)*orientation(2, 0)
    );

    // Parallel shift and rotation of goal position
    dx = cos(phi(0))*dx + sin(phi(0))*position(1) - d1x;
    dz -= d1z;

    // Check length of setting goal
    if (sqrt(dx*dx + dz*dz) > d2 + d3 + d4) {
        ROS_FATAL_STREAM("Solution NOT exists!!");
        return false;
    }

    px = dx;
    pz = dz;
    startAlpha = alpha;
    uint i = 0;
    double offset = 1;

    // Solve IK in plane space
    while (alpha > startAlpha - M_PI/2 && alpha < startAlpha + M_PI/2) {

        alpha = startAlpha + 0.01*(-1 + 2*sign)*i;
        sign = !sign;
        i++;

        dx = px - d4*sin(alpha);
        dz = pz - d4*cos(alpha);

        for (uint k = 0; k < 2; ++k) { 
            valid = true;
            cosPhi3 = (dx*dx + dz*dz - d2*d2 - d3*d3)/(2*d2*d3);
            if (cosPhi3 > 1) phi(2) = 0;
            else if (cosPhi3 < -1) phi(2) = M_PI;
            else phi(2) = offset*atan2(sqrt(1 - cosPhi3*cosPhi3), cosPhi3);
            offset *= -1;
            // else phi(2) = offset[i]*atan2(sqrt(1 - cosPhi3*cosPhi3), cosPhi3);

            phi(1) = atan2(dx, dz) - atan2(d3*sin(phi(2)), d2 + d3*cos(phi(2)));
            phi(3) = alpha - phi(2) - phi(1);

            // Constraints checking 
            for (uint j = 1; j < 4; ++j) {
                valid = valid && (jointMinAngles[j] <= phi(j) && phi(j) <= jointMaxAngles[j]);
            }
     
            if (valid) {
                solutions.push_back(phi);
                // ROS_INFO_STREAM("jointValues: (" << 
                //     phi(0) << ", " <<
                //     phi(1) << ", " <<
                //     phi(2) << ", " <<
                //     phi(3) << ", " <<
                //     phi(4) << ")");
                // ROS_INFO_STREAM("Alpha: (" << alpha << ")");
                return true;
            }
        }
    }
    return false;
}

void ArmKinematics::calcOrientationMatrix(double phi1, double phi5, double alpha, matrix::Matrix<double, 3, 3> & orientation)
{
    orientation(0, 0) = cos(phi1)*cos(alpha)*cos(phi5) - sin(phi1)*sin(phi5);
    orientation(0, 1) = -cos(phi1)*cos(alpha)*sin(phi5) - sin(phi1)*cos(phi5);
    orientation(0, 2) = cos(phi1)*sin(alpha);

    orientation(1, 0) = sin(phi1)*cos(alpha)*cos(phi5) - cos(phi1)*sin(phi5);
    orientation(1, 1) = -sin(phi1)*cos(alpha)*sin(phi5) + cos(phi1)*cos(phi5);
    orientation(1, 2) = sin(phi1)*sin(alpha);

    orientation(2, 0) = -sin(alpha)*cos(phi5);
    orientation(2, 1) = sin(alpha)*sin(phi5);
    orientation(2, 2) = cos(alpha);
}

bool ArmKinematics::solveIK(const Vector3d & position, const double phi5, const double alpha, std::vector<JointValues> & solutions)
{

    double phi1 = atan2(position(1), (position(0) - d0x));
	if (phi1 < jointMaxAngles[0] || phi1 > jointMinAngles[0]) phi1 += M_PI;
    matrix::Matrix<double, 3, 3> orientation;

    // ROS_INFO_STREAM("phi: (" << phi1 << ", " << phi5 << ", " << alpha << ")");
    // ROS_INFO_STREAM("pos: (" << position(0) << ", " << position(1) << ", " << position(2) << ")");

    calcOrientationMatrix(phi1, phi5, alpha, orientation);

    return ArmKinematics::solveIK(position, orientation, solutions);
}

bool ArmKinematics::solveIK(const Vector3d & position, const double phi1, const double phi5, const double alpha, std::vector<JointValues> & solutions)
{
    matrix::Matrix<double, 3, 3> orientation;
    calcOrientationMatrix(phi1, phi5, alpha, orientation);
    return ArmKinematics::solveIK(position, orientation, solutions);
}
bool ArmKinematics::solveIK(const Vector3d & position, const Vector3d & rotationAngles, std::vector<JointValues> & solutions)
{
    matrix::Matrix<double, 3, 3> orientation;
    calcOrientationMatrix(rotationAngles(0), rotationAngles(1), rotationAngles(2), orientation);
    return ArmKinematics::solveIK(position, orientation, solutions);
}
bool ArmKinematics::solveIK(const Pose & position, std::vector<JointValues> & solutions)
{
    matrix::Matrix<double, 3, 3> orientation;
    calcOrientationMatrix(position.orientation(0), position.orientation(1), position.orientation(2), orientation);
    return ArmKinematics::solveIK(position.position, orientation, solutions);
}

void ArmKinematics::calcKinematicsParams(const Vector3d & linearVelocities, const JointValues & jointAngles, JointValues & jointAngVel, JointValues & jointAngAcc)
{
    double dJ[2][2];
    double invJ[2][2];

    // Inverse Jacobian
    invJ[0][0] = sin(jointAngles(2) + jointAngles(1)) / (d2*sin(jointAngles(2)));
    invJ[0][1] = cos(jointAngles(2) + jointAngles(1)) / (d2*sin(jointAngles(2)));
    invJ[1][0] = -(d2*sin(jointAngles(1)) + d3*sin(jointAngles(2) + jointAngles(1))) / (d2*d3*sin(jointAngles(2)));
    invJ[1][1] = -(d2*cos(jointAngles(1)) + d3*cos(jointAngles(2) + jointAngles(1))) / (d2*d3*sin(jointAngles(2)));

    // Calculate angular velocity of joint
    jointAngVel(1) = invJ[0][0]*linearVelocities(0) + invJ[0][1]*linearVelocities(2);
    jointAngVel(2) = invJ[1][0]*linearVelocities(0) + invJ[1][1]*linearVelocities(2);
    jointAngVel(3) = - jointAngVel(1) - jointAngVel(2);

    // Check joint velocities
    // for (uint i = 1; i <= 3; ++i) {
    //     if (jointAngVel(i) < jointMinVel[i] || jointAngVel(i) > jointMaxVel[i])
    //         jointAngVel(i) = (jointAngVel(i) > 0) - (jointAngVel(i) < 0);
    // }

    // Jacobian derivative
    dJ[0][0] = -d3*(jointAngVel(1) + jointAngVel(2))*sin(jointAngles(1) + jointAngles(2)) - d2*jointAngVel(1)*sin(jointAngles(1));
    dJ[0][1] = -d3*(jointAngVel(1) + jointAngVel(2))*sin(jointAngles(1) + jointAngles(2));
    dJ[1][0] = -d3*(jointAngVel(1) + jointAngVel(2))*cos(jointAngles(1) + jointAngles(2)) - d2*jointAngVel(1)*cos(jointAngles(1));
    dJ[1][1] = -d3*(jointAngVel(1) + jointAngVel(2))*cos(jointAngles(1) + jointAngles(2));

    // Calculate acceleration of joint
    jointAngAcc(1) = -invJ[0][0]*(dJ[0][0]*jointAngVel(1) + dJ[0][1]*jointAngVel(2)) - invJ[0][1]*(dJ[1][0]*jointAngVel(1) + dJ[1][1]*jointAngVel(2));
    jointAngAcc(2) = -invJ[1][0]*(dJ[0][0]*jointAngVel(1) + dJ[0][1]*jointAngVel(2)) - invJ[1][1]*(dJ[1][0]*jointAngVel(1) + dJ[1][1]*jointAngVel(2));
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
    double dx = d2*sin(jointAngles(1)) + d3*sin(jointAngles(1) + jointAngles(2)) + (d4 - 0.105)*sin(alpha) + d1x;
    double dz = d2*cos(jointAngles(1)) + d3*cos(jointAngles(1) + jointAngles(2)) + (d4 - 0.105)*cos(alpha) + d1z;

    // x-coordinate
    resultPosition(0) = (cos(alpha)*cos(phi1)*cos(phi5) - sin(phi1)*sin(phi5)) * position(0)
                        - (cos(alpha)*cos(phi1)*sin(phi5) + sin(phi1)*cos(phi5)) * position(1)
                        + (sin(alpha)*cos(phi1)) * position(2)
                        + dx*cos(phi1) + d0x;
    // y-coordinate
    resultPosition(1) = (cos(phi1)*sin(phi5) + cos(alpha)*sin(phi1)*cos(phi5)) * position(0)
                        + (cos(phi1)*cos(phi5) - cos(alpha)*sin(phi1)*sin(phi5)) * position(1)
                        + (sin(alpha)*sin(phi1)) * position(2)
                        + dx*sin(phi1);
    // z-coordinate
    resultPosition(2) =  - (sin(alpha)*cos(phi5)) * position(0)
                        + (sin(alpha)*sin(phi5)) * position(1)
                        + (cos(alpha)) * position(2)
                        + dz + d0z;

    ROS_INFO_STREAM("Position: (" << 
                resultPosition(0) << ", " <<
                resultPosition(1) << ", " <<
                resultPosition(2) << ")");

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
