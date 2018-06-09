clear();
directory = get_absolute_file_path("NumIKHardTest.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1);

// Finding maximum y-rotation (theta angle);
function ang = calcMaxRot(pos)
    disp(pos)
    P = pos(1:3) - d0;
    sgn = sign(P(1));
    q1 = atan(P(2), sgn*P(1));      //TODO: Check

    d23 = d2(3) + d3(3);
    // Shift and rotate coordinate system
    goal = [
        P(1)*cos(q1) + P(2)*sin(q1);
        -P(1)*sin(q1) + P(2)*cos(q1);
        P(3);
    ] - d1;
    

    d = d4(3) + griperLength;
    if d23 + d < norm(goal) then
        disp("SOLUTION not exists!");
        return;
    end

    cosq4 = (norm(goal)^2 - d23^2 - d^2)/(2*d23*d);
//    disp(cosq4)
    q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
    q2 = atan(goal(1), goal(3)) - atan(d*sin(q4), d23 + d*cos(q4));

    q3 = 0;
    if q4 > jointLimits(4, 2) then
        q4 = jointLimits(4, 2);
        disp("goal:" + string(goal));
        d34 = norm(d3 + [d*sin(q4); 0; d*cos(q4)]);
        disp("d34:" + string(d34));
        cosq3 = (norm(goal)^2 - d2(3)^2 - d34^2)/(2*d2(3)*d34);
        q3 = atan(sqrt(1 - cosq3^2), cosq3);
        q2 = atan(goal(1), goal(3)) - atan(d34*sin(q3), d2(3) + d34*cos(q3));
        q3 = q3 - atan(d*sin(q4), d3(3) + d*cos(q4));
        disp("q4 > jointLimits(4, 2)");
    end
    if q4 < jointLimits(4, 1) then
        q4 = jointLimits(4, 1);
        d34 = norm(d3 + [d*sin(q4); 0; d*cos(q4)]);
        cosq3 = (norm(goal)^2 - d2(3)^2 - d34^2)/(2*d2(3)*d34);
        q3 = atan(sqrt(1 - cosq3^2), cosq3);
        q2 = atan(goal(1), goal(3)) - atan(d34*sin(q3), d2(3) + d34*cos(q3));
        q3 = q3 - atan(d*sin(q4), d3(3) + d*cos(q4));
        disp("q4 < jointLimits(4, 1)");
    end

    if q2 > jointLimits(2, 2) then
        disp("goal:" + string(goal));
        q2 = jointLimits(2, 2);
        d34 = goal - d2(3)*[sin(q2); 0; cos(q2)];
        disp("d34:" + string(d34));
        cosq4 = (norm(d34)^2 - d3(3)^2 - d^2)/(2*d3(3)*d);
        q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
        q23 = atan(d34(1), d34(3)) - atan(d*sin(q4), d3(3) + d*cos(q4));
        q3 = q23 - q2;
        disp("q2 > jointLimits(2, 2)");
    end
    if q2 < jointLimits(2, 1) then
        q2 = jointLimits(2, 1);
        d34 = goal - d2(3)*[sin(q2); 0; cos(q2)];
        cosq4 = (norm(d34)^2 - d3(3)^2 - d^2)/(2*d3(3)*d);
        q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
        q23 = atan(d34(1), d34(3)) - atan(d*sin(q4), d3(3) + d*cos(q4));
        q3 = q23 - q2;
        disp("q2 < jointLimits(2, 2)");
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

// Desired orientation
//      θ   Ψ
ori = [%pi, 0];
// Ψ -- not use

initConfiguration = [0.312209; 0.0875557; -0.0260085; ori(1); 0];
endConfiguration = [0.312209; 0.0875557; -0.0960085; ori(1); 0];
maxVel = 0.05; maxAccel = 0.1;
timeStep = 0.05;

[time, velTra, posTra] = workSpaceTraj(initConfiguration, endConfiguration, maxAccel, maxVel, timeStep);
poses = posTra;
T = length(time);

// TEMP calculation
e1 = 0.1; off1 = 0.1;
e3 = 0.2; off3 = 0;
e2 = (e3 + e1)/2; off2 = (off1 + off3)/2;
A = [...
    e3^2, e3, 1;
    e2^2, e2, 1;
    2*e3, 1, 0
];
B = [off3; off2; 0];

// ax^2 + bx + c
a1 = -20; b1 = 4; c1 = -0.1;
a2 = 20; b2 = -8; c2 = 0.8;

shft1 = 0.1;
shft2 = 0.2;

////// ** Convert work space to joint space
maxRot = zeros(DOF, 1);
info = [];
q_traj = [];
q_traj2 = [];
err = 0; e_iter = 0;// error
offset = 0;
offsets = [];
errors = [];

curConf = poses(:, 1);
ang = calcMaxRot(curConf);
initialAngle = zeros(DOF, 1);
initialAngle(2) = ang(1);
initialAngle(3) = ang(2);
initialAngle(4) = ang(3);
for i = 1:T;
    curConf = poses(:, i);
    sgn = sign(curConf(1));
    ang = calcMaxRot(curConf);
    theta = ang(1) + ang(2) + ang(3);

    // e = θm - θd
    err = sgn*(theta - ori(1));
    // Stabilizing regulator; 0.1 - offset constant
    if (err < 0) then
        offset = 0.1;
    elseif (err < 0.2) then
        offset = 2.5*(err - 0.2)^2 + err;
    elseif (err > 0.2) then
        offset = err;
    end;
//    disp("err");
//    disp(err)
    errors = [errors, err];
    offsets = [offsets, offset];
//    offset = 0;
//    if (theta > ori(1) - 0.1) then
//        offset = (theta + 0.1 - ori(1))^2;
//    else
//        offset = 0;
//    end;
    poses(4, i) = theta - sgn*offset;
    poses(5, i) = 0;
    [q_curr, err, info] = numIK(poses(:, i), initialAngle);
    q_traj = [q_traj, q_curr];
//    q_traj2 = [q_traj2, (IK(poses(1:3, i), getR(0, poses(4, i), 0), [0, 1]))];
    if (err <> 0) then
        disp("Caught ERROR!");
        disp("Information: ");
        disp(poses(:, i));
        disp(offset)
        return;
    end
    initialAngle = q_curr;
end
animation(q_traj, size(q_traj, 2), 30, ax);

//// plot trajectories
if length(get_figure_handle(100)) then
    f = get_figure_handle(100);
    clf(100);
    a = f.children;
else
    f = figure(100);
    a = f.children;
end
for i = 1:5
    n = size(q_traj, 2);
    subplot(3, 2, i);

    plot(time(1:n), q_traj(i, :));

    e = gce();
    e.children.thickness = 2;
    e.children.foreground = color("blue");
    e.children.line_style = 1;

//    plot(time(1:n), q_traj2(i + 1, :));
//    e = gce();
//    e.children.thickness = 2;
//    e.children.foreground = color("orange");
//    e.children.line_style = 4;

//    legend("Numerical", "Analitic");

    a.font_size = 1;
    a.filled = "on";
    a.box = "off";
    a.background = -2;
end

