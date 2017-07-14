cmake_minimum_required(VERSION 2.8.3)
project(arm_manipulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    brics_actuator
    arm_kinematics
    actionlib
    control_msgs
    sensor_msgs
    trajectory_msgs
    trajectory_generator
)

catkin_package()

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(test_manipulation tests/test.cpp src/youbot_manipulator.cpp)
target_link_libraries(test_manipulation
    ${catkin_LIBRARIES}
)
add_executable(move_gripper tests/move_gripper.cpp src/youbot_manipulator.cpp)
target_link_libraries(move_gripper
    ${catkin_LIBRARIES}
)
add_executable(translation_test tests/translationTest.cpp)
target_link_libraries(translation_test
    ${catkin_LIBRARIES}
)
add_executable(trajectory_test tests/trajectory_test.cpp src/youbot_manipulator.cpp)
target_link_libraries(trajectory_test
    ${catkin_LIBRARIES}
)

add_executable(move_by_camera src/move_by_camera.cpp src/youbot_manipulator.cpp)
target_link_libraries(move_by_camera
    ${catkin_LIBRARIES}
)

add_library(youbot_manipulator
    src/youbot_manipulator.cpp
)
target_link_libraries(youbot_manipulator
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)