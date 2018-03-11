clear;
directory = "/home/egor/vrepWS/src/red_manipulation_step/trajectory_generator/scilab/";
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);

function [time, vel, coord, rot] = workSpaceTraj(v_i, v_e, maxAccel, maxVel, timeStep)
    // initial and end point
    p_i = v_i(1:3); p_e = v_e(1:3);
    p_diff = p_e - p_i;

    t1 = maxVel/maxAccel;
    t2 = norm(p_diff)/maxVel;
    t3 = t2 + t1;
    d = normalize(p_diff);
    diraction = sign(norm(p_diff));

    // rotation trajectory
    theta_i = v_i(4); theta_e = v_e(4);
    psi_i = v_i(5); psi_e = v_e(5);
    rotVel = [theta_e - theta_i; psi_e - psi_i]/t3;

    function absVel = velFunc(x)
	disp("t:",x,t);
        if x >= 0 & x < t1 then
            absVel = diraction*maxAccel*t;
            return
        end
        if x >= t1 & x < t2 then
            absVel = diraction*maxVel;
            return
        end
        if x >= t2 & x < t3 then
            absVel = diraction*(maxVel - maxAccel*(t - t2));
            return
        end
        absVel = 0;
    endfunction

    function rv = rotVelFunc(x)
        if x >= 0 & x < t3 then
            rv = rotVel;
        end
        rv = [0; 0];
    endfunction

    // IF t2 < t1 then ERROR
    if t2 < t1 then
        disp("t2 < t1");
        return;
    end

    vel = []; currVel = [];
    coord = []; currCoord = p_i;
    rot = []; currRot = [theta_i; psi_i];
    time = (-5*timeStep:timeStep:t3 + 5*timeStep) + 5*timeStep;
    for t = -5*timeStep:timeStep:t3 + 5*timeStep
        currVel = d*velFunc(t);
        vel = [vel, currVel];
        coord = [coord, currCoord];
        rot = [rot, currRot]
        currCoord = currCoord + timeStep/2 * (currVel + d*velFunc(t + timeStep));
        currRot = currRot + timeStep/2 * (rotVelFunc(t) + rotVelFunc(t + timeStep));
    end
endfunction

//function q = workSpaceToJoint(time, v)
//    q = [];
//endfunction

