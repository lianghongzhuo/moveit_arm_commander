cmake_minimum_required(VERSION 3.0.2)
project(arm_motion_service)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ArmMotionSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package( DEPENDS message_runtime )
