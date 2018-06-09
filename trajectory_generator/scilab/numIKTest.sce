clear();
directory = get_absolute_file_path("numIKTest.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1)

// Desired orientation
//      θ   Ψ
ori = [%pi, 0];
// Ψ -- not use
v = [0.3; 0; 0.0; ori(1); 0];
// Finding maximum y-rotation (theta angle);
function ang = calcMaxRot(pos)
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
    

    if d23 + d4(3) < norm(goal) then
        disp("SOLUTION not exists!");
        return;
    end

    cosq4 = (norm(goal)^2 - d23^2 - d4(3)^2)/(2*d23*d4(3));
    q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
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
    if q4 < jointLimits(4, 1) then
        q4 = jointLimits(4, 1);
        d34 = norm(d3 + [d4(3)*sin(q4); 0; d4(3)*cos(q4)]);
        cosq3 = (norm(goal)^2 - d2(3)^2 - d34^2)/(2*d2(3)*d34);
        q3 = atan(sqrt(1 - cosq3^2), cosq3);
        q2 = atan(goal(1), goal(3)) - atan(d34*sin(q3), d2(3) + d34*cos(q3));
        q3 = q3 - atan(d4(3)*sin(q4), d3(3) + d4(3)*cos(q4));
//        disp("theta_max (correct): " + string(q2+q3+q4));
    end

    if q2 > jointLimits(2, 2) then
        q2 = jointLimits(2, 2);
        d34 = goal - d2(3)*[sin(q2); 0; cos(q2)];
        cosq4 = (norm(d34)^2 - d3(3)^2 - d4(3)^2)/(2*d3(3)*d4(3));
        q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
        q23 = atan(d34(1), d34(3)) - atan(d4(3)*sin(q4), d3(3) + d4(3)*cos(q4));
        q3 = q23 - q2;
//        disp("[calcMaxRot] q2: Out of range!");
    end
    if q2 < jointLimits(2, 1) then
        q2 = jointLimits(2, 1);
        d34 = goal - d2(3)*[sin(q2); 0; cos(q2)];
        cosq4 = (norm(d34)^2 - d3(3)^2 - d4(3)^2)/(2*d3(3)*d4(3));
        q4 =  sgn*atan(sqrt(1 - cosq4^2), cosq4);
        q23 = atan(d34(1), d34(3)) - atan(d4(3)*sin(q4), d3(3) + d4(3)*cos(q4));
        q3 = q23 - q2;
//        disp("[calcMaxRot] q2: Out of range!");
    end
    ang = [q2; q3; q4; q2 + q3 + q4 - sgn*0.1];
endfunction
h = figure(1);
a = gca();
// input - [joint values, axis, 1/0 - axis configure
visualizationFK([0; 0; 0; 0; 0], a, 1);

//ang = calcMaxRot(v)
ang = [...
    1.5338837  
    0.0330047  
    1.9437422
];
visualizationFK([0; ang(1); ang(2); ang(3); 0], a, 0);

// Initial condition
//q_i = zeros(DOF, 1);
//q_i(2) = ang(1);
//q_i(3) = ang(2);
//q_i(4) = ang(3);
//v(4) = ang(4);
//disp(v);

// Solve IK (numerical method)
//[q_curr, e, info] = numIK(v, q_i);

//visualizationFK(q_curr, a, 1);
