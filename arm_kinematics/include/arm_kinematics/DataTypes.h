#ifndef DATA_TYPES
#define DATA_TYPES

#define SUPPORT_STDIOSTREAM
#include <matrix/math.hpp>

typedef matrix::Vector3<double> Vector3d;
typedef matrix::Vector<double, 5> JointValues;

struct Pose
{
    Vector3d position;
    Vector3d orientation;
    // Orientation(0) - theta; Angle of y axis (2 - 4 joint roataion)
    // Orientation(1) - psi;   Angle of z axis (gripper rotation)
};

#endif