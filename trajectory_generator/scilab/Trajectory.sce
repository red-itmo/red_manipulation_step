clear;
directory = get_absolute_file_path("Trajectory.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);

function [time, vel, coord, rot] = workSpaceTraj(startPose, endPose, maxAccel, maxVel, timeStep)
    // initial and end point
    p_i = startPose(1:3); p_e = endPose(1:3);
    p_diff = p_e - p_i;

    t1 = maxVel/maxAccel;
    t2 = norm(p_diff)/maxVel;
    t3 = t2 + t1;
    movementDirection = normalize(p_diff);
    directionSign = sign(norm(p_diff));

    // rotation trajectory
    theta_i = startPose(4); theta_e = endPose(4);
    psi_i = startPose(5); psi_e = endPose(5);
    rotVel = [theta_e - theta_i; psi_e - psi_i]/t3;

    function absVel = getVel(x)
        if x >= 0 & x < t1 then
            absVel = directionSign*maxAccel*t;
            return
        end
        if x >= t1 & x < t2 then
            absVel = directionSign*maxVel;
            return
        end
        if x >= t2 & x < t3 then
            absVel = directionSign*(maxVel - maxAccel*(t - t2));
            return
        end
        absVel = 0;
    endfunction

    function rv = getRotVel(x)
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
    // rot = []; currRot = [theta_i; psi_i];
    time = (-5*timeStep:timeStep:t3 + 5*timeStep) + 5*timeStep;
    for t = -5*timeStep:timeStep:t3 + 5*timeStep
        currVel = movementDirection*getVel(t);
        vel = [vel, currVel];
        coord = [coord, currCoord];
        // rot = [rot, currRot]
        currCoord = currCoord + timeStep/2 * (currVel + movementDirection*getVel(t + timeStep));
        currRot = currRot + timeStep/2 * (getRotVel(t) + getRotVel(t + timeStep));
    end
endfunction
