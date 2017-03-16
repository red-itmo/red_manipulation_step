cmake_minimum_required(VERSION 2.8.3)
project(manipulation_control_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arm_kinematics
    roscpp
    std_srvs
    sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulation_control_node
#  CATKIN_DEPENDS arm_kinematics roscpp std_srvs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_library(manipulation_control_node
    src/ManipulationControlNode.cpp
)

add_executable(start_node 
	src/main.cpp
	src/ManipulationControlNode.cpp
)
target_link_libraries(start_node
    ${catkin_LIBRARIES}
)