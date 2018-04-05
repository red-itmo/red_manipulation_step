cmake_minimum_required(VERSION 2.8.3)
project(arm_manipulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    brics_actuator
    arm_kinematics
    actionlib
    control_msgs
    sensor_msgs
    trajectory_msgs
    trajectory_generator
    red_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(manipulation_test tests/ManipulationTest.cpp)
add_executable(move_gripper tests/MoveGripper.cpp)
add_executable(trajectory_test tests/TrajectoryTest.cpp)

add_library(youbot_manipulator
    src/youbot_manipulator.cpp
    src/TrajectoryMove_fn.cpp
)
target_link_libraries(move_gripper
    youbot_manipulator
    ${catkin_LIBRARIES}
)
target_link_libraries(manipulation_test
    youbot_manipulator
    ${catkin_LIBRARIES}
)
target_link_libraries(trajectory_test
    youbot_manipulator
    ${catkin_LIBRARIES}
)