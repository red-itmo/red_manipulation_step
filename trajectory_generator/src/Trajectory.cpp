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
    if (time >= 0 && time < t1)
        return directionSign * maxAccel * time;
    if (time >= t1 && time < t2)
        return directionSign * maxVel;
    if (time >= t2 && time <= t3)
        return directionSign * (maxVel - maxAccel * (time - t2));
    return 0;
}

double Trajectory::getTrajectoryTime()
{
    return t3;
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
    //if no difference between start and end positions
    if(t2==0){
        ROS_WARN_STREAM("Start and end pose coincide!");
        t2=t1+0.01;
    }
    t3 = t2 + t1;


    ROS_INFO_STREAM("[Trajectory] Time variables (" << t1 << ", " << t2 << ", " << t3 << ")");

    if (t2 < t1)
    {
        ROS_FATAL_STREAM("[Trajectory]t2<t1. Try to decrease max speed");
        return;
    }
    Vector3d currCoord = startPose.position;
    Vector3d currVel;

    for (double currentTime = 0; currentTime < t3+2*timeStep; currentTime += timeStep)
    {
        currVel = movementDirection * getVel(currentTime);
        velTra.push_back(currVel);
        posTra.push_back(currCoord);
        time.push_back(currentTime);
        currCoord += timeStep / 2 * (currVel + movementDirection * getVel(currentTime + timeStep));
    }
    ROS_INFO_STREAM("[trajectory]trj is generated, number of steps:"<<posTra.size());
}

void Trajectory::convertWorkSpaceToJointSpace(const Pose startPose, Pose endPose, const double timeStep)
{
    JointValues currJntAng, currJntAngVel, currJntAngAcc;
    JointValues prevAngles, currAngles, nextAngles, anglesDiff, zeroVec;
    prevAngles.setAll(-1000);
    ArmKinematics solver;
    double prevTime, currTime, nextTime;

    Vector3d rotVel, startRot, endRot, currRot, currPosition;
    double startAlpha, endAlpha, alpha, alphaVel;

    //filling poses vector
    std::vector<Pose> poses;
    Pose pose;
    for (int i = 0; i < posTra.size(); ++i) {
        pose.position = posTra[i];
        poses.push_back(pose);
        // pose.position.print();
        // pose.orientation.print();
    }

    double err = 0, offset = 0, theta;
    Pose curConf;
    Vector3d ang;
    JointValues curRot, angle;
    angle(1) = M_PI / 3;
    angle(2) = M_PI / 6;
    angle(3) = M_PI / 6;

    for (size_t i = 0; i < poses.size(); ++i)
    {
        curConf = poses[i];
        ang = solver.calcMaxRot(curConf.position);
        //if error occured
        if(ang(0)==-1000)
            return;

        theta = ang(0) + ang(1) + ang(2);
        offset = 0;
        if (theta > startPose.orientation(0) - 0.1)
            offset = pow(theta + 0.1 - startPose.orientation(0), 2);
        else
            offset = 0;
        curConf.orientation(0) = theta - 0.1 - offset;
        curRot = solver.numericalIK(curConf, angle);
        //if error has occured
        if(curRot(0)==-1000 || curRot(0)==-2000){
            ROS_WARN_STREAM("Trajectory soluion not found!");
            return;
        }
        makeYoubotArmOffsets(curRot);
        //if the first iteration
        if(prevAngles(0)==-1000)
            prevAngles=curRot;
        currJntAngVel = (curRot - prevAngles) / timeStep;
        prevAngles = curRot;
        qdotTra.push_back(currJntAngVel);
        qTra.push_back(curRot);

        angle = solver.prevNumIKAngle;
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
        currJntAngVel = qdotTra[p];
        //TODO acceleration
        //currJntAngAcc = qdotdotTra[p];

        for (size_t i = 0; i < 5; ++i) {
            point.positions.push_back(currJntAng(i));
            // ROS_DEBUG_ONCE("[trajectory]Publishing trajectory angles:");
            // ROS_DEBUG("[trajectory]%f",currJntAng(i));
            point.velocities.push_back(currJntAngVel(i));
            point.accelerations.push_back(currJntAngAcc(i));
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

void Trajectory::generatePowers(const int n, double* powers) {
    powers[0] = 1.0;
    for (int i = 2; i <= n; i++)
        powers[i] = powers[i - 1] * powers[i];
}

void Trajectory::quinticSpline(const Vector3d & start_pos, const Vector3d & end_pos, const std::vector<double> & tr_time, const Vector3d & start_vel, const Vector3d & end_vel)
{
    /* JTRAJ Compute a joint space trajectory

    [Q,QD,QDD] = JTRAJ(Q0, QF, M) is a joint space trajectory Q (MxN) where the joint
    coordinates vary from Q0 (1xN) to QF (1xN).  A quintic (5th order) polynomial is used
    with default zero boundary conditions for velocity and acceleration.
    Time is assumed to vary from 0 to 1 in M steps.  Joint velocity and
    acceleration can be optionally returned as QD (MxN) and QDD (MxN) respectively.
    The trajectory Q, QD and QDD are MxN matrices, with one row per time step,
    and one column per joint.

    [Q,QD,QDD] = JTRAJ(Q0, QF, M, QD0, QDF) as above but also specifies
    initial QD0 (1xN) and final QDF (1xN) joint velocity for the trajectory.

    [Q,QD,QDD] = JTRAJ(Q0, QF, T) as above but the number of steps in the
    trajectory is defined by the length of the time vector T (Mx1).

    [Q,QD,QDD] = JTRAJ(Q0, QF, T, QD0, QDF) as above but specifies initial and
    final joint velocity for the trajectory and a time vector.

    Notes::
    - When a time vector is provided the velocity and acceleration outputs
    are scaled assumign that the time vector starts at zero and increases
    linearly.*/
    //tscal allows to change the shape of spline somehow
    std::vector<double> t;
    double tscal;
    if (tr_time.size() > 1)
    {
        tscal = *std::max_element(tr_time.begin(), tr_time.end());
        for (auto i : tr_time) {
            t.push_back(i / tscal);
        }
    }
    else
    {
        tscal = 1;
        for (int i = 0; i <= tr_time[0] - 1; i++)
            t.push_back(i / (tr_time[0] - 1)); // normalized time from 0 -> 1
    }

    Vector3d A = 6.0 * (end_pos - start_pos) - 3.0 * (end_vel + start_vel) * tscal;
    Vector3d B = -15.0 * (end_pos - start_pos) + (8.0 * start_vel + 7.0 * end_vel) * tscal;
    Vector3d C = 10.0 * (end_pos - start_pos) - (6.0 * start_vel + 4.0 * end_vel) * tscal;
    Vector3d E = start_vel * tscal; // as the t vector has been normalized
    Vector3d F = start_pos;

    Vector3d pos, vel, acc;
    //starting from the second time to make spline continious
    for (int i = 1; i < t.size(); i++)
    {
        double T[6] = {t[i], t[i], t[i], t[i], t[i], t[i]};
        generatePowers(5, T);
        for (int j = 0; j < 3; j++)
        {
            pos(j) = T[5] * A(j) + T[4] * B(j) + T[3] * C(j) + T[1] * E(j) + T[0] * F(j);
            vel(j) = (T[4] * 5 * A(j) + T[3] * 4 * B(j) + T[2] * 3 * C(j) + T[0] * E(j)) / tscal;
            acc(j) = (T[3] * 20 * A(j) + T[2] * 12 * B(j) + T[1] * 6 * C(j)) / pow(tscal, 2);
        }
        posTra.push_back(pos);
        velTra.push_back(vel);
        accTra.push_back(acc);
    }
}

void Trajectory::mstraj(double maxVel, double dt, double maxAccel, const Pose & startPose, const std::vector<Pose> & segmentsPose)
{
    /* TRAJ = MSTRAJ(P, QDMAX, TSEG, Q0, DT, TACC, QD0, QDF) is a trajectory
    (KxN) for N axes moving simultaneously through M segment.  Each segment
    is linear motion and polynomial blends connect the segments.  The axes
    start at Q0 (1xN) and pass through M-1 via points defined by the rows of
    the matrix P (MxN), and finish at the point defined by the last row of P.
    The  trajectory matrix has one row per time step, and one column per
    axis.  The number of steps in the trajectory K is a function of the
    number of via points and the time or velocity limits that apply.

    - P (MxN) is a matrix of via points, 1 row per via point, one column
    per axis.  The last via point is the destination.
    - QDMAX (1xN) are axis speed limits which cannot be exceeded,
    - TSEG (1xM) are the durations for each of the K segments
    - Q0 (1xN) are the initial axis coordinates
    - DT is the time step
    - TACC (1x1) is the acceleration time used for all segment transitions
    - TACC (1xM) is the acceleration time per segment, TACC(i) is the acceleration
    time for the transition from segment i to segment i+1.  TACC(1) is also
    the acceleration time at the start of segment 1.

    TRAJ = MSTRAJ(SEGMENTS, QDMAX, Q0, DT, TACC, QD0, QDF) additionally
    specifies the initial and final axis velocities (1xN).

    Notes::
    - Only one of QDMAX or TSEG can be specified, the other is empty.
    - The path length K is a function of the number of via points, Q0, DT
    and TACC.
    - The final via point P(end,:) is the destination.
    - The motion has M segments from Q0 to P(1,:) to P(2,:) ... to P(end,:).
    - All axes reach their via points at the same time. */

    ROS_INFO_STREAM("calculating trj... Start position:(" << startPose.position(0) << " " << startPose.position(1) << " " << startPose.position(2) << ")");
    ROS_INFO("via points:");
    std::vector<Vector3d> segments;
    for (int i = 0; i < segmentsPose.size(); i++) {
        segments.push_back(segmentsPose[i].position);
        ROS_INFO_STREAM("(" << segmentsPose[i].position(0) << " " << segmentsPose[i].position(1) << " " << segmentsPose[i].position(2) << ")");
    }
    std::vector<double> tsegment;   //unspecified time, may be used in future
    Vector3d qdmax;
    qdmax.setAll(maxVel);   //setting the same velocities for all axes: x,y,z
    //One may set different speeds for axes
    std::vector<double> Tacc;
    //in order to meet acceleration constraints acceleration time is increased
    //meeting velocity limit is guaranteed(99%)
    //real acceleration may be a little higher than limit one
    //because only acceleration per axis is somehow limited unlike the sum of axes
    Tacc.push_back(maxVel * 1 / maxAccel);  //and the same acceleration times

    if ((qdmax(0) != 0 || qdmax(1) != 0 || qdmax(2) != 0) && tsegment.size() > 0)
    {
        ROS_FATAL("Must specify either qdmax or tsegment, but not both");
        return;
    }

    // set the initial conditions
    Vector3d prev_pos = startPose.position, next_pos, prev_vel, dq, cur_vel, qdf, cur_pos = startPose.position;
    std::vector<double> tr_time;

    double tacc2, taccx, tacc, tseg;

    for (int seg = 0; seg < segments.size(); seg++)
    {
        if (Tacc.size() > 1)
            tacc = Tacc[seg];
        else
            tacc = Tacc[0];
        tacc = std::ceil(tacc / dt) * dt;
        tacc2 = std::ceil(tacc / 2 / dt) * dt;
        if (seg == 0)
            taccx = tacc2;
        else
            //required time between via points is usually more than at the first step
            taccx = tacc;

        // estimate travel time
        // could better estimate distance travelled during the blend
        next_pos = segments[seg];    // current target
        dq = next_pos - prev_pos;    // total distance to move this segment
        tseg = dq.norm() / maxVel + taccx;
        //const speed motion time should be more than acceleration plus decceleration time
        if (tseg <= 2 * tacc)
            tseg = 2 * tacc + tacc / 2;
        // linear velocity from qprev to qnext
        cur_vel = dq / (tseg - taccx);
        // add the blend polynomial
        tr_time.clear();
        for (double i = 0; i <= tacc * 3; i += dt)
            tr_time.push_back(i);
        quinticSpline(prev_pos, prev_pos + tacc2 * cur_vel, tr_time, prev_vel, cur_vel);
        cur_pos = posTra.back();

        // add the linear part, from tacc/2+dt to tseg-tacc/2
        for (double t = taccx + dt; t < tseg - tacc; t += dt)
        {
            Vector3d zeroAcc;
            cur_pos += cur_vel * dt;    //linear step
            // std::cout<<" cur_pos:"<<cur_pos(0)<<" "<<cur_pos(1)<<" "<<cur_pos(2)<<"\n";
            posTra.push_back(cur_pos);
            velTra.push_back(cur_vel);
            accTra.push_back(zeroAcc);
        }
        prev_pos = posTra.back();    // next target becomes previous target
        prev_vel = cur_vel;
    }
    // add the final blend
    tr_time.clear();
    for (double i = 0; i < tacc * 3; i += dt)
        tr_time.push_back(i);
    // std::cout<<"LASTcur_pos:\n"<<prev_pos<<"end_pos:\n"<<next_pos<< "prev_vel:\n"<<prev_vel<< "\n";
    quinticSpline(prev_pos, next_pos, tr_time, prev_vel, qdf);

    //filling time vector according to the size of position vector
    for (int i = 0; i < posTra.size(); i++)
        time.push_back(dt * i);
    t3=posTra.size()*dt;
    ROS_INFO_STREAM("[trajectory]trj is generated, number of steps:" << posTra.size());
}