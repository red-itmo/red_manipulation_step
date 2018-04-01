clear();
directory = get_absolute_file_path("NumIKHardTest.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1);

// Finding maximum y-rotation (theta angle);
function ang = calcMaxRot(pos)
    position = pos(1:3);
    //q1= 0; // temp
    //d4z = d4(3); // temp - length with gripper
    d23 = d2(3) + d3(3);
    ang = [0; 0];
    goal = position - d0 - d1;    // First goal

    if d23 + d4(3) < norm(goal) then
        disp("SOLUTION not exists!");
        return;
    end

    cosq4 = (norm(goal)^2 - d23^2 - d4(3)^2)/(2*d23*d4(3));
    q4 = atan(sqrt(1 - cosq4^2), cosq4);
    q2 = atan(goal(1), goal(3)) - atan(d4(3)*sin(q4), d23 + d4(3)*cos(q4));
    
    q3 = 0;
    if q4 > jointLimits(4, 2) then
        q4 = jointLimits(4, 2);
        d34 = norm(d3 + [d4(3)*sin(q4); 0; d4(3)*cos(q4)]);
        cosq3 = (norm(goal)^2 - d2(3)^2 - d34^2)/(2*d2(3)*d34);
        q3 = atan(sqrt(1 - cosq3^2), cosq3);
        q2 = atan(goal(1), goal(3)) - atan(d34*sin(q3), d2(3) + d34*cos(q3));
        q3 = q3 - atan(d4(3)*sin(q4), d3(3) + d4(3)*cos(q4));
//        disp("theta_max (correct): " + string(q2+q3+q4));
    end

    if q2 > jointLimits(4, 2) then
        disp("q2: Out of range!");
    end
    ang = [q2; q3; q4];
endfunction

if length(get_figure_handle(1)) then
    h = get_figure_handle(1);
    ax = h.children;
else
    h = figure(1);
    ax = h.children;
end

initConfiguration = [-0.3; 0; 0.1; -%pi; 0];
endConfiguration = [0.3; 0; -0.05; %pi; 0];
maxVel = 0.1; maxAccel = 0.5;
timeStep = 0.02;

[time, velTra, posTra, rotTra] = workSpaceTraj(initConfiguration, endConfiguration, maxAccel, maxVel, timeStep);
poses = [posTra; rotTra];
T = length(time);

// Calculate rotation trajectory
rotTraj = [];
confFirst = poses(:, 1);
confEnd = poses(:, T);
angles1 = calcMaxRot(confFirst);
angles2 = calcMaxRot(confEnd);
theta1 = angles1(1) + angles1(2) + angles1(3); theta2 = angles2(1) + angles2(2) + angles2(3);
if theta1 > %pi then
    theta1 = %pi;
end
if theta2 > %pi then
    theta2 = %pi;
end

rotTraj = (theta2 - theta1)/time(T) * time + theta1;


////// ** Convert work space to joint space
maxRot = zeros(DOF, 1);
info = [];
q_traj = [];
q_traj2 = [];
err = 0; e_iter = 0;// error
offset = 0;
offsets = [];

for i = 1:T;
	curConf = poses(:, i);
    ang = calcMaxRot(curConf);
    maxRot = zeros(DOF, 1);
    maxRot(2) = %pi/3;
    maxRot(3) = %pi/6;
    maxRot(4) = %pi/6;
    theta = ang(1) + ang(2) + ang(3); //rotTraj(i);

    offset = 0;
    if (theta > %pi - 0.1) then
        offset = (theta + 0.1 - %pi)^2;
    else
        offset = 0;
    end;
    poses(4, i) = theta - 0.1 - offset;
    [q_curr, err, info] = numIK(poses(:, i), maxRot);
    q_traj = [q_traj, q_curr];
    q_traj2 = [q_traj2, (IK(poses(1:3, i), getR(0, poses(4, i), 0), [0, 1]))];
    if (err <> 0) then
        disp("Caught ERROR!");
        disp("Information: ");
        disp(poses(:, i));
        return;
    end
end
animation(q_traj, size(q_traj, 2), 1, ax);

//// plot trajectories
if length(get_figure_handle(100)) then
    f = get_figure_handle(100);
    clf(100);
    a = f.children;
else
    f = figure(100);
    a = f.children;
end
for i = 1:3
    n = size(q_traj, 2);
    subplot(3, 1, i);
    
    plot(time(1:n), q_traj(i + 1, :));
	
    e = gce();
    e.children.thickness = 2;
    e.children.foreground = color("blue");
    e.children.line_style = 1;
    
    plot(time(1:n), q_traj2(i + 1, :));
    e = gce();
    e.children.thickness = 2;
    e.children.foreground = color("orange");
    e.children.line_style = 4;

    legend("Numerical", "Analitic");

    a.font_size = 1;
    a.filled = "on";
    a.box = "off";
    a.background = -2;
end

