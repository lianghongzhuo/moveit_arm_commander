cmake_minimum_required(VERSION 3.0.2)
project(moveit_arm_commander)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_python_setup()
catkin_package()
