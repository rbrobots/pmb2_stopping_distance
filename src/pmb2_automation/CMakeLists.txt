cmake_minimum_required(VERSION 3.0.2)
project(pmb2_automation)

find_package(catkin REQUIRED)


catkin_package()


catkin_install_python(PROGRAMS stopping_distance.py
		       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

