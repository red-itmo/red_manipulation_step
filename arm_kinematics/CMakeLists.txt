cmake_minimum_required(VERSION 2.8.3)
project(arm_kinematics)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
	message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# Declare the service files to be built
add_service_files(
    FILES
    PoseArray.srv
    CameraFrame.srv
    ManipulatorPose.srv
)

add_message_files(
  FILES
  CertesianPose.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES arm_kinematics
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(arm_kinematics
    src/ArmKinematics.cpp
)

#install binary and lib
install(TARGETS arm_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
