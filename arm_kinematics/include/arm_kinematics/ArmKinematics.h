#ifndef ARM_KINEMATICS
#define ARM_KINEMATICS

#include <ros/ros.h>
#include <arm_kinematics/DataTypes.h>
#include <vector>


void makeYoubotArmOffsets(JointValues & jointAngles);
void makeKinematicModelOffsets(JointValues & jointAngles);

class ArmKinematics {

    public:

        ArmKinematics();
        ~ArmKinematics();

        /*
        * Froward Kinematics width offset for camera where
        * position - position of object at the camera frame
        * return position of new frame named base
        */
        Vector3d transformFromFrame5ToFrame0(const JointValues & jointAngles, const Vector3d & position);

        /*
        * Inverse Kinematics function where
        * position - position of end-effector
        * orientation - rotation matrix 3x3
        * return joint angles if solution was found
        */
        bool solveIK(const Vector3d & position, matrix::Matrix<double, 3, 3> & orientation, std::vector<JointValues> & solutions);
        bool solveIK(const Vector3d & position, const double phi5, const double alpha, std::vector<JointValues> & solutions);
        bool solveIK(const Vector3d & position, const double phi1, const double phi5, const double alpha, std::vector<JointValues> & solutions);
        bool solveIK(const Vector3d & position, const Vector3d & rotationAngles, std::vector<JointValues> & solutions);
        bool solveIK(const Pose & position, std::vector<JointValues> & solutions);

        // Calc velocities and accelerations
        void calcKinematicsParams(const Vector3d & linearVelocities, const JointValues & jointAngles, JointValues & jointAngVel, JointValues & jointAngAcc);

    private:

        void calcOrientationMatrix(double phi1, double phi5, double alpha, matrix::Matrix<double, 3, 3> & orientation);
};

#endif
