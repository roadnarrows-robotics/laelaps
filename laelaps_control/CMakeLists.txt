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

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    Alarms.msg
    AmbientLightSensorCaps.msg
    AuxPwr.msg
    Caps.msg
    Dimensions.msg
    DutyCycle.msg
    Dynamics.msg
    Gpio.msg
    IlluminanceState.msg
    ImuAlt.msg
    ImuCaps.msg
    MotorCtlrHealth.msg
    MotorHealth.msg
    Path2D.msg
    Pose2DStamped.msg
    PowertrainCaps.msg
    ProductInfo.msg
    RangeState.msg
    RobotStatusExtended.msg
    RobotTrajectory2D.msg
    RobotTrajectoryPoint2D.msg
    ToFSensorCaps.msg
    Velocity.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    ConfigGpio.srv
    EStop.srv
    Freeze.srv
    GetCaps.srv
    GetIlluminance.srv
    GetImu.srv
    GetImuAlt.srv
    GetProductInfo.srv
    GetRange.srv
    IsAlarmed.srv
    IsDescLoaded.srv
    ReadGpio.srv
    Release.srv
    ReloadConfig.srv
    ResetEStop.srv
    SetAuxPwr.srv
    SetDutyCycles.srv
    SetRobotMode.srv
    SetVelocities.srv
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

add_executable(laelaps_teleop
  src/laelaps_teleop/laelaps_teleop.cpp
  src/laelaps_teleop/laelaps_teleop_main.cpp)

target_link_libraries(laelaps_control 
  ${rnr-laelaps_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(laelaps_teleop 
  ${rnr-laelaps_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(laelaps_control laelaps_control_gencpp)
add_dependencies(laelaps_teleop laelaps_control_gencpp)

install(
  PROGRAMS 
    scripts/laelaps_alarms
    scripts/laelaps_cam
    scripts/laelaps_estop
    scripts/laelaps_move_backward
    scripts/laelaps_move_forward
    scripts/laelaps_panel
    scripts/laelaps_ranges
    scripts/laelaps_rviz
    scripts/laelaps_spin_left
    scripts/laelaps_spin_right
    scripts/laelaps_twist
    scripts/laelaps_stop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
