# Installation

1. run git clone
```
https://github.com/red-itmo/red_manipulation_step.git
```

2. Place "matrix" library to /usr/include (https://github.com/PX4/Matrix)

3. run ```catkin_make```

# How to run?

To start trajectory test run
```
roslaunch arm_manipulation trajectory_test.launch
```

To turn off and go to initial position or turn on motors start goToInitAndRelax node
```
rosrun arm_manipulation goToInitAndRelax
```
and run
```
rosservice call /arm_manipulation/switchOffMotors
```
or
```
rosservice call /arm_manipulation/switchOnMotors
```
<!-- 2. Start cv - manipulation control node
```
rosrun manipulation_control_node start_node
```

3. Send message for starting
```
rosservice call /table_feasible
``` -->