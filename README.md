# Installation

1. To copy this project run
```
git clone https://github.com/red-itmo/red_manipulation_step.git
```

2. Place "matrix" library to /usr/include (https://github.com/PX4/Matrix)

3. run ```catkin_make```

# How to run?

To start trajectory test run
```
roslaunch arm_manipulation trajectory_test.launch
```

## Node goToIntAndRelax
Start node
```
rosrun arm_manipulation goToInitAndRelax
```
To turn off and go to initial position and/or turn on motors run
```
rosservice call /arm_manipulation/switchOffMotors
rosservice call /arm_manipulation/switchOnMotors
```

## Node start_manipulation
Start
```
roslaunch arm_manipulation start_manipulation.launch
```
To move gripper to some position or to move from one point to another run
```
rosservice call /manipulator_pose
rosservice call /MoveLine
```
<!-- 2. Start cv - manipulation control node
```
rosrun manipulation_control_node start_node
```

3. Send message for starting
```
rosservice call /table_feasible
``` -->