# pmb2_stopping_distance

To execute:

* Install PMB2 package as shown in PDF:
* import pmb2_stopping_distance into pmb2_public_ws
* catkin build in pmb2_public_ws
* source .devel 
* roslaunch pmb2_2dnav_gazebo pmb2_mapping.launch public_sim:=true world:=large_corridor
* rosrun pmb2_automation stopping_distance.py


Current issues:

* CMakeLists.txt does not understand when the 'scripts' folder is used.
* Need to get odometry feedback from robot
