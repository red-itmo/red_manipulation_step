#include <arm_kinematics/ArmKinematics.h>

int main(int argc, char ** argv)
{
    ArmKinematics solver;
    JointValues q_i, q;
    Pose p;

    p.position(0) = 0.36;
    p.position(1) = 0.0;
    p.position(2) = 0.1;
    p.orientation(0) = 2.0;
    p.orientation(1) = 0;

    q = solver.numericalIK(p, q_i);
    std::cout << q << std::endl;
    return 0;
}