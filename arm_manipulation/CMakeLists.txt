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
    message_generation
    std_msgs
    geometry_msgs
)

add_service_files(
    FILES
    MoveLine.srv
)

generate_messages(
	DEPENDENCIES
	arm_kinematics
	geometry_msgs
	std_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	geometry_msgs
	message_runtime
	std_msgs 
	arm_kinematics	
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(test_manipulation tests/test.cpp)
add_executable(move_gripper tests/move_gripper.cpp)
add_executable(camera_calibration tests/CameraCalibration.cpp)
add_executable(trajectory_test tests/trajectory_test.cpp)
add_executable(grasping_test tests/GraspingTest.cpp)
add_executable(move_by_camera src/move_by_camera.cpp)

add_library(youbot_manipulator
    src/youbot_manipulator.cpp
    src/TrajectoryMove_fn.cpp
)
target_link_libraries(move_gripper
    youbot_manipulator
    ${catkin_LIBRARIES}
)
target_link_libraries(test_manipulation
    youbot_manipulator
    ${catkin_LIBRARIES}
)
target_link_libraries(trajectory_test
    youbot_manipulator
    ${catkin_LIBRARIES}
)
target_link_libraries(camera_calibration
    ${catkin_LIBRARIES}
)
target_link_libraries(grasping_test
    ${catkin_LIBRARIES}
)
target_link_libraries(move_by_camera
    youbot_manipulator
    ${catkin_LIBRARIES}
)

