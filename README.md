#Installation

1. run git clone 
```
https://github.com/red-itmo/red_manipulation_step.git
```

2. Place "matrix" library to /usr/include (https://github.com/PX4/Matrix)

3. run ```catkin_make```

<!-- # How to run?

1. Start manipulation node.
```
roslaunch arm_manipulation start_manipulation.launch
```

2. Start cv - manipulation control node
```
rosrun manipulation_control_node start_node
```

3. Send message for starting
```
rosservice call /table_feasible -->
```