cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    arm_kinematics
    trajectory_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trajectory_generator
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(trajectory_generator
    src/Trajectory.cpp
    src/TrajectoryGenerator.cpp
)
target_link_libraries(trajectory_generator
    ${catkin_LIBRARIES}
)

add_executable(work_traj_test
    tests/workSpaceTrajectoryTest.cpp
    src/Trajectory.cpp
)
target_link_libraries(work_traj_test
    ${catkin_LIBRARIES}
)
add_executable(joint_traj_test
    tests/jointSpaceTrajectoryTest.cpp
    src/Trajectory.cpp
)
target_link_libraries(joint_traj_test
    ${catkin_LIBRARIES}
)
