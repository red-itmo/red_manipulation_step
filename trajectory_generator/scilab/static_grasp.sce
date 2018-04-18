clear;
directory = get_absolute_file_path("static_grasp.sce");
exec(directory + "math.sce", -1);
exec(directory + "kinematic.sce", -1);
exec(directory + "Trajectory.sce", -1);
function dT =Rot(a,speed)
	z=0;
	dT=[cos(a),-sin(a),0,speed*cos(a);
	    sin(a),cos(a),0,speed*sin(a);
	    0,0,1,z;
	    0,0,0,1];
endfunction



curr_coord=[0.26;0;0.1;1];//object's start point
coord_arr=[curr_coord];
dphi=0;//trajectory bending
phi_sum=0;
speed=0.01;//speed of object, m/s
t_end=3;//prediction time, s
for i = 0:t_end
	next_coord=Rot(dphi,speed)*curr_coord;
	curr_coord=next_coord;
	coord_arr=[coord_arr,curr_coord];
	phi_sum=phi_sum+dphi;
	phi_sum=modulo(phi_sum,2*%pi);
	//break;
end
initConfiguration = [0.26; 0; 0.1; %pi; 0];
endConfiguration = [curr_coord(1:3); %pi; phi_sum];
maxVel = 0.05; maxAccel = 0.1;
timeStep = 0.05;
[time, velTra, posTra] = workSpaceTraj(initConfiguration, endConfiguration, maxAccel, maxVel, timeStep);
plot(coord_arr(1,:),coord_arr(2,:));
