#ifndef ARM_KINEMATICS
#define ARM_KINEMATICS

#include <ros/ros.h>
#include <arm_kinematics/DataTypes.h>
#include <vector>


void makeYoubotArmOffsets(JointValues & jointAngles);
void makeKinematicModelOffsets(JointValues & jointAngles);
bool checkAngles(const JointValues & jointAngles);

class ArmKinematics {
public:

    ArmKinematics();
    ~ArmKinematics();

    //solves forward kinematics
    Vector3d transformFromFrame5ToFrame0(const JointValues & jointAngles, const Vector3d & position);

    // Solve IK for first and
    bool solveSpaceIK(Vector3d & position, matrix::SquareMatrix<double, 3> & orientation, JointValues & jointAngles, double & alpha);
    bool solveSpaceIK(Vector3d & position, const Vector3d & orientation, JointValues & jointAngles, double & alpha);
    bool solvePlaneIK(Vector3d position, const double alpha, JointValues & jointAngles, const int configuration);

    bool solveIK(const Vector3d & position, matrix::SquareMatrix<double, 3> & orientation, JointValues & jointAngles, const int configuration);
    bool solveIK(const Vector3d & position, const double phi5, const double alpha, JointValues & jointAngles, const int configuration);
    bool solveIK(const Vector3d & position, const Vector3d & orientation, JointValues & jointAngles, const int configuration);
    bool solveIK(const Pose & position, JointValues & jointAngles, const int configuration);

    bool solveFullyIK(const Pose & position, JointValues & jointAngles);

    // Calc velocities and accelerations
    void calcKinematicsParams(const Vector3d & linearVelocities, const JointValues & jointAngles, JointValues & jointAngVel, JointValues & jointAngAcc);

    matrix::SquareMatrix<double, 3> calcOrientationMatrix(double phi1, double phi5, double alpha);

    matrix::SquareMatrix<double, 5> Jacobian(const JointValues &angles);
    Vector3d calcMaxRot(const Vector3d & position);
    //wrapper for transformFromFrame5ToFrame0
    Vector3d ForwardKin(const JointValues &angles);
    JointValues numericalIK(const Pose &pose, JointValues q);
    JointValues prevNumIKAngle;
};

#endif
