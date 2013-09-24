cmake_minimum_required(VERSION 2.8.3)
project(laelaps_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  industrial_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

# RDK, DHP TODO Create rnrpkgConfig.cmake
# find_package(rnr
#   REQUIRED COMPONENTS laelaps Dynamixel botsense netmsgs rnr_appkit rnr
# )
find_library(
  rnr_LIBRARIES
  NAMES mot rnr
  PATHS /prj/lib /prj/lib/rnr /usr/local/lib /usr/local/lib/rnr
)

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    test_msg.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    test_srv.srv
)

#add_action_files(
#DIRECTORY action 
#FILES 
#Calibrate.action
#)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include 
  /prj/include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(laelaps_control src/laelaps_control.cpp)

target_link_libraries(
  laelaps_control 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(laelaps_control laelaps_control_gencpp)

install(
  PROGRAMS 
    scripts/laelaps_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

