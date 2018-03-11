clear();
directory = "/home/egor/vrepWS/src/red_manipulation_step/trajectory_generator/scilab/";
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1)


//q = [-%pi/4; -%pi/4; %pi/4; %pi/2; -%pi/2]
//[P, R] = FK(q)

//config = [0, 1];
//a = IK(P, R, config)

//q_i = [0; 0; 0; 0; 0];
//[P, R] = FK([0; %pi/2; 0; 0; 0]);
//v = [P; %pi/2; %pi];
//q = numIK(v, q_i)

v_i = [0.3; -0.1; 0; %pi/2; 0];
v_e = [0.3; 0; 0; %pi/2; 0];
maxVel = 0.1; maxAccel = 1;
timeStep = 0.01;

[t, v, x, r] = workSpaceTraj(v_i, v_e, maxAccel, maxVel, timeStep);
s_traj = [x; r];

// ** Convert work space to joint space
q_i = zeros(DOF, 1);
q_traj = [];
q_min = []; q_max = []; n_trj = [];
n = 0;
// error
e = 0;
for i = 1:length(t);
    [q_curr, e] = numIK(s_traj(:, i), q_i);
    q_i = q_curr;
//    if () 

//    n = int(q_curr/(2*%pi));
//    qMinCurr = n*2*%pi + jointLimits(:, 1);
//    qMaxCurr = n*2*%pi + jointLimits(:, 2);
//    n_trj = [n_trj, n];
//    q_min = [q_min, qMinCurr];
//    q_max = [q_max, qMaxCurr];
//    for j = 1:DOF
//        if (q_curr(j) > qMaxCurr(j)) then
//            q_curr(j) = q_curr(j) - 2*%pi;
//        elseif (q_curr(j) < qMinCurr(j)) then
//            q_curr(j) = q_curr(j) + 2*%pi;
//        end
//    end
//    q_curr = q_curr - n*2*%pi;

    q_traj = [q_traj, q_curr];
end
// ** end **

T = length(t);
for i = 1:DOF
    subplot(DOF, 1, i)
//    plot(t, n_trj(i, :)*2*%pi, 'k--');
//    plot(t, q_max(i, :), 'r--');
//    plot(t, q_min(i, :), 'r--');
//    plot([t(1), t(T)], [jointLimits(i, 1), jointLimits(i, 1)], 'k--');
//    plot([t(1), t(T)], [jointLimits(i, 2), jointLimits(i, 2)], 'k--');
    plot(t, q_traj(i, :));

    a = gca();
//    a.data_bounds = [t(1), -n*%pi; t(T), %pi];
end
