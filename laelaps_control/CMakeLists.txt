cmake_minimum_required(VERSION 2.8.3)
project(laelaps_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  geometry_msgs
  industrial_msgs
  nav_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-laelaps_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-laelaps
  REQUIRED
)

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    Caps.msg
    Dimensions.msg
    Dynamics.msg
    Gpio.msg
    MotorCtlrHealth.msg
    MotorHealth.msg
    Path2D.msg
    Pose2DStamped.msg
    ProductInfo.msg
    RobotStatusExtended.msg
    Velocity.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    ConfigGpio.srv
    EStop.srv
    Freeze.srv
    GetCaps.srv
    GetProductInfo.srv
    Go.srv
    IsAlarmed.srv
    IsDescLoaded.srv
    ReadGpio.srv
    Release.srv
    ReloadConfig.srv
    ResetEStop.srv
    SetRobotMode.srv
    Stop.srv
    WriteGpio.srv
)

#add_action_files(
#DIRECTORY action 
#FILES 
#Calibrate.action
#)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    industrial_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
    ${rnr-laelaps_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(laelaps_control
  src/laelaps_control.cpp
  src/laelaps_control_main.cpp)

target_link_libraries(laelaps_control 
  ${rnr-laelaps_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(laelaps_control laelaps_control_gencpp)

install(
  PROGRAMS 
    scripts/laelaps_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
