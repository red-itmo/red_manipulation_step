directory = get_absolute_file_path("kinematic.sce");
exec(directory + "math.sce");

DOF = 5;
jointLimits = [
    -169, 169;
    -65,  90;
    -151, 146;
    -102, 102;
    -167, 167
]
jointLimits = jointLimits * %pi/180;

d0 = [0.024; 0; 0.096];
d1 = [0.033; 0; 0.019];
d2 = [0; 0; 0.155];
d3 = [0; 0; 0.135];
d4 = [0; 0; 0.13];
griperLength = 0.105;
d5 = [0; 0; griperLength];

// P - position [x; y; z]
// R - rotation.[psi1, theta, psi2]
// atan(y, x) == atan2
// config = [0|%pi, -1|1]
function q=IK(P, R, config)
    q = zeros(DOF, 1)

    // Space IK0
    P = P - d0;
    if config(1) == %pi then
        if (atan(-P(2), P(1)) + config(1) < jointLimits(1, 2)) then
            q(1) = atan(-P(2), P(1)) + config(1);
        elseif (atan(-P(2), P(1)) - config(1) > jointLimits(1, 1))
            q(2) = atan(-P(2), P(1)) - config(1);
        else
            disp("q1: Cant reach this configuration.");
            return;
        end
    else
        q(1) = atan(-P(2), P(1));
    end

    psi1 = q(1);
//    if R(3, 3) > 10e-4 then
    theta = atan(cos(psi1)*R(1, 3) - sin(psi1)*R(2, 3), R(3, 3));
//    else
//        theta = 
//    end
    psi2 = - atan(sin(psi1)*R(1, 1) + cos(psi1)*R(2, 1), sin(psi1)*R(1, 2) + cos(psi1)*R(2, 2));
    q(5) = psi2;

    // Rotate and shift the frame pg - plane goal
    d = d4;// + d5
    pg = Rz(-q(1))*P - d1;
    if (norm(pg) - d2(3) - d3(3) - d(3) > 10e-4) then
        disp("Goal cant be reached.");
        q = zeros(DOF, 1);
        return;
    end
    pg = pg - [sin(theta); 0; cos(theta)]*d(3);

    // Plane IK
    if (norm(pg) - d2(3) - d3(3) > 10e-4) then
        disp("Solution not found");
        return;
    end

    cosq3 = (pg(1)^2 + pg(3)^2 - d2(3)^2 - d3(3)^2)/(2*d2(3)*d3(3));
    if abs(1 - abs(cosq3)) < 0.1 then
        cosq3 = sign(cosq3);
    end
    q(3) = atan(sqrt(1 - cosq3^2), config(2)*cosq3);
    q(2) = atan(pg(1), pg(3)) - atan(d3(3)*sin(q(3)), d2(3) + d3(3)*cos(q(3)));
    q(4) = theta - q(2) - q(3);

    // Check all angles
    for i = 1:DOF
        if (q(i) > jointLimits(i, 2) | q(i) < jointLimits(i, 1)) then
            disp("q" + string(i) + ": Out of range.");
            return;
        end
    end
endfunction

// TODO modidy
// rot_i = [theta_i, psi2_i]
// err - error code
// err - 1: Solution not exists
// err - 2: Angles out of range
function [q, err, other] = numIK(pose, q_i)
    // Error code:
    // 1 - Any solution cant be found 

    dq = zeros(DOF, 1);
    q = zeros(DOF, 1);
    q = q_i;
    other = [];
    iter_num = 50;

    k = 0.05;
    iter = 0;
    difference = pose - [FK(q); q(2) + q(3) + q(4); q(5)];

    while (norm(difference) > 10e-10 & iter < iter_num) // (norm(e) > 10e-4) & 
        j = J(q);
        dq = inv(j'*j + k^2*eye(DOF, DOF))*j'*difference;

        q = q + dq;
        difference = (pose - [FK(q); q(2) + q(3) + q(4); q(5)]);
        iter = iter + 1;
    end

    if (iter > iter_num-1) then
        q = zeros(DOF, 1);
        err = 1;
        disp("Solution cant be found.");
        return;
    end
    [valid_limits, joint_num] = checkJointLimits(q);
    if ~valid_limits then
        disp("[num IK] q" + string(joint_num) + ": Out of range.");
        err = 2;
        return;
    end
    
    q = round(q*10000)/10000;
    err = 0;
endfunction
function q = normalizeAngles(q_raw)
    [q_min, q_max, n] = getShiftedAnglesBounds(q_raw);
    q = q_raw - n*2*%pi;
endfunction
// n - level of shift
function [q_min, q_max, n] = getShiftedAnglesBounds(q_raw)
    n = int(q_raw/(2*%pi));
    q_min = n*2*%pi + jointLimits(:, 1);
    q_max = n*2*%pi + jointLimits(:, 2);

    for i = 1:DOF
        if (q_raw(i) > q_max(i)) then
            q_max(i) = q_max(i) + 2*%pi;
            q_min(i) = q_min(i) + 2*%pi;
            n(i) = n(i) + 1;
        elseif (q_raw(i) < q_min(i)) then
            q_max(i) = q_max(i) - 2*%pi;
            q_min(i) = q_min(i) - 2*%pi;
            n(i) = n(i) - 1;
        end
    end
endfunction
function [valid, joint_num] = checkJointLimits(q)
    q = normalizeAngles(q);
    valid = %t;
    joint_num = 0;

    for i = 1:DOF
        if (q(i) > jointLimits(i, 2) | q(i) < jointLimits(i,1)) then
            joint_num = i;
            valid = %f;
            return;
        end
    end
endfunction

function [P, R] = FK(q)
    P = zeros(3);
    R = zeros(3, 3);

    psi1 = q(1); 
    theta = q(2) + q(3) + q(4);
    psi2 = q(5);

    d = d4(3);// + d5(3);
    plane = [
        d2(3)*sin(q(2)) + d3(3)*sin(q(2) + q(3)) + d*sin(q(2) + q(3) + q(4));
        0;
        d2(3)*cos(q(2)) + d3(3)*cos(q(2) + q(3)) + d*cos(q(2) + q(3) + q(4));
    ]
    P = d0 + Rz(psi1) * (plane + d1);
    R = getR(psi1, theta, psi2);

endfunction

function R = getR(psi1, theta, psi2)
    R = Rz(psi1)*Ry(-theta)*Rz(psi2);
endfunction
function j = J(q)
    j = zeros(5, 5);
    d = d4;// + d5;

    j(1, 1) = -sin(q(1))*(d1(1)+d2(3)*sin(q(2))+d3(3)*sin(q(3)+q(2))+d(3)*sin(q(4)+q(3)+q(2)));
    j(2, 1) = -cos(q(1))*(d1(1)+d2(3)*sin(q(2))+d3(3)*sin(q(3)+q(2))+d(3)*sin(q(4)+q(3)+q(2)));
    j(3, 1) = 0;
    j(4, 1) = 0;
    j(5, 1) = 0;

    j(1, 2) = cos(q(1))*(d(3)*cos(q(4)+q(3)+q(2))+d3(3)*cos(q(3)+q(2))+d2(3)*cos(q(2)));
    j(2, 2) = -sin(q(1))*(d2(3)*cos(q(2))+d3(3)*cos(q(3)+q(2))+d(3)*cos(q(4)+q(3)+q(2)));
    j(3, 2) = -d(3)*sin(q(4)+q(3)+q(2))-d3(3)*sin(q(3)+q(2))-d2(3)*sin(q(2));
    j(4, 2) = 1;
    j(5, 2) = 0;

    j(1, 3) = cos(q(1))*(d(3)*cos(q(4)+q(3)+q(2))+d3(3)*cos(q(3)+q(2)));
    j(2, 3) = -sin(q(1))*(d3(3)*cos(q(3)+q(2))+d(3)*cos(q(4)+q(3)+q(2)));
    j(3, 3) = -d(3)*sin(q(4)+q(3)+q(2))-d3(3)*sin(q(3)+q(2));
    j(4, 3) = 1;
    j(5, 3) = 0;

    j(1, 4) = d(3)*cos(q(1))*cos(q(4)+q(3)+q(2));
    j(2, 4) = -d(3)*sin(q(1))*cos(q(4)+q(3)+q(2));
    j(3, 4) = -d(3)*sin(q(4)+q(3)+q(2));
    j(4, 4) = 1;
    j(5, 4) = 0;

    j(1, 5) = 0;
    j(2, 5) = 0;
    j(3, 5) = 0;
    j(4, 5) = 0;
    j(5, 5) = 1;

endfunction
function q_out = shiftAngles(q)
    q_out = modulo(q, 4*%pi);
endfunction

// For testing
function cq = kinToYoubot(q)
    cq(1) = q(1) - jointLimits(1, 1);
    cq(2) = q(2) - jointLimits(2, 1);
    cq(3) = q(3) - jointLimits(3, 2);
    cq(4) = q(4) - jointLimits(4, 1);
    cq(5) = q(5) - jointLimits(5, 1);
endfunction
function cq = youbotToKin(q)
    cq(1) = q(1) + jointLimits(1, 1);
    cq(2) = q(2) + jointLimits(2, 1);
    cq(3) = q(3) + jointLimits(3, 2);
    cq(4) = q(4) + jointLimits(4, 1);
    cq(5) = q(5) + jointLimits(5, 1);
endfunction


// VISUALIZATION
function visualizationFK(q, axis, formating);

    a = sca(axis);
    delete(a.children);
    d = d4(3);// + d5(3);
    p = [d0(1) + d1(1); d0(3) + d1(3)];
    p = [p, p(:, 1) + [d2(3)*sin(q(2)); d2(3)*cos(q(2))]];
    p = [p, p(:, 2) + [d3(3)*sin(q(2) + q(3)); d3(3)*cos(q(2) + q(3))]];
    p = [p, p(:, 3) + [d*sin(q(2) + q(3) + q(4)); d*cos(q(2) + q(3) + q(4))]];

    for i = 1:3
        plot2d([p(1, i), p(1, i+1)], [p(2, i), p(2, i+1)]);
        e = gce();
        // Line style
        e.children.thickness = 3;
        e.children.foreground = color("orange");

        // Mark style
        e.children.mark_style = 9;
        e.children.mark_size = 1;
        e.children.mark_background = color("black");
        e.children.mark_foreground = color("orange");
    end

    if formating then
        bound = d0(3) + d1(3) + d2(3) + d3(3) + d;
        a.font_size = 2;
        a.isoview = "on";
        a.filled = "on";
        a.box = "off";
        a.data_bounds = [-bound, -bound; bound, bound];
        a.background = -2;
        xgrid;
    end

endfunction
function animation(q_traj, n, sleep_time, axis)
    visualizationFK(q_traj(:, 1), axis, 1);

    a = 0;
//    a = input("Start? ");

    for i = 1:n
        drawlater();
        d = d4(3);// + d5(3);
        p = [d0(1) + d1(1); d0(3) + d1(3)];
        p = [p, p(:, 1) + [d2(3)*sin(q_traj(2, i)); d2(3)*cos(q_traj(2, i))]];
        p = [p, p(:, 2) + [d3(3)*sin(q_traj(2, i) + q_traj(3, i)); d3(3)*cos(q_traj(2, i) + q_traj(3, i))]];
        p = [p, p(:, 3) + [d*sin(q_traj(2, i) + q_traj(3, i) + q_traj(4, i)); d*cos(q_traj(2, i) + q_traj(3, i) + q_traj(4, i))]];
    
        axis.children(3).children.data = [p(1, 1), p(2, 1); p(1, 2), p(2, 2)];
        axis.children(2).children.data = [p(1, 2), p(2, 2); p(1, 3), p(2, 3)];
        axis.children(1).children.data = [p(1, 3), p(2, 3); p(1, 4), p(2, 4)];
        sleep(sleep_time);
        drawnow();
    end
endfunction
