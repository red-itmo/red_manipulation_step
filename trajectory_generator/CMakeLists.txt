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
    LIBRARIES trajectory_generator trajectory_lib
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
add_executable(work_traj_test tests/workSpaceTrajectoryTest.cpp)
add_executable(joint_traj_test tests/jointSpaceTrajectoryTest.cpp)
add_executable(multiseg_test tests/multisegTest.cpp)

add_library(trajectory_lib src/Trajectory.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(trajectory_generator
    src/TrajectoryGenerator.cpp
)
target_link_libraries(trajectory_generator
    trajectory_lib
    ${catkin_LIBRARIES}
)
target_link_libraries(work_traj_test
    trajectory_lib
    ${catkin_LIBRARIES}
)
target_link_libraries(joint_traj_test
    trajectory_lib
    ${catkin_LIBRARIES}
)
target_link_libraries(multiseg_test
    trajectory_lib
    ${catkin_LIBRARIES}
)