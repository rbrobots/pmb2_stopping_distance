cmake_minimum_required(VERSION 3.0.2)
project(pmb2_automation)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   gazebo_msgs
   message_generation
)

add_message_files(
  FILES
  Data.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)


catkin_install_python(PROGRAMS stopping_distance.py
		       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

