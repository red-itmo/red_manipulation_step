clear();
directory = get_absolute_file_path("numIKTest.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1)

z_i = 0.1;
z_e = 0;
v = [0.36; 0; z_i; %pi; 0];
// Finding maximum y-rotation (theta angle);
function ang = calc_q_i(v)
    x = v(1:3);
    q1 = 0; // temp
    d = d4(3); // temp - length with gripper
    d23 = d2(3) + d3(3);
    ang = [0; 0];
    g = x - d0 - d1;    // First goal
    if d23 + d < norm(g) then
        disp("SOLUTION not exists!");
        return;
    end

    cosq4 = (norm(g)^2 - d23^2 - d^2)/(2*d23*d);
    q4 = atan(sqrt(1 - cosq4^2), cosq4);
    q2 = atan(g(1), g(3)) - atan(d*sin(q4), d23 + d*cos(q4));
    disp("theta_max: " + string(q2+q4));

    q3 = 0;
    if q4 > jointLimits(4, 2) then
        q4 = jointLimits(4, 2);
        d34 = norm(d3 + [d*sin(q4); 0; d*cos(q4)]);

        cosq3 = (norm(g)^2 - d2(3)^2 - d34^2)/(2*d2(3)*d34);
        q3 = atan(sqrt(1 - cosq3^2), cosq3);
        q2 = atan(g(1), g(3)) - atan(d34*sin(q3), d2(3) + d34*cos(q3));

        q3 = q3 - atan(d*sin(q4), d3(3) + d*cos(q4));
        disp("theta_max (correct): " + string(q2+q3+q4));
    end

    if q2 > jointLimits(4, 2) then
        disp("q2: Out of range!");
    end
    ang = [q2; q3; q4];
endfunction
h = figure(1);
a = gca();
// input - [joint values, axis, 1/0 - axis configure
visualizationFK([0; 0; 0; 0; 0], a, 1);

ang = calc_q_i(v);
visualizationFK([0; ang(1); ang(2); ang(3); 0], a, 0);

// Initial condition
q_i = zeros(DOF, 1);
q_i(2) = %pi/3;
q_i(3) = %pi/6;
q_i(4) = %pi/6;
v(4) = sum(ang) - 0.01;

// Solve IK (numerical method)
[q_curr, e, info] = numIK(v, q_i);

visualizationFK(q_curr, a, 1);
