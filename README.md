# How to run?

# 1. Start manipulation node.

```
roslaunch arm_manipulation start_manipulation.launch
```

# 2. Start cv - manipulation control node
```
rosrun manipulation_control_node start_node
```

# 3. Send message for starting

rosservice call /table_feasible