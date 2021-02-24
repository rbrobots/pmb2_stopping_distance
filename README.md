# pmb2_stopping_distance

<strong>Summary</strong>

This script tests the positive linear and angular stopping distance of a robot. 
The current ranges tested for linear are 0.1 m/s to 1.0 m/s.
The current ranges tested for angular are 1.0 m/s to 2.0 m/s.
These can be modified by editing the 'execute_test_scripts' function in stopping_distance.py

<strong>To execute:</strong>

* Install PMB2 package as shown in PDF:
* import pmb2_stopping_distance into pmb2_public_ws
* catkin build in pmb2_public_ws
* source ./devel/setup.bash
* roslaunch pmb2_2dnav_gazebo pmb2_mapping.launch public_sim:=true world:=large_corridor
* ./data_recorder.py
* rosrun pmb2_automation stopping_distance.py

<strong>Current issues:</strong>

* CMakeLists.txt does not understand when the 'scripts' folder is used.
* Publishers require delay before execution. Need to find out an alternative method to get this working.

<strong>To add:</strong>

Test Scenarios
* Negative linear and negative angular movement
* Combined linear and angular movement
* Obstacles - obstacles in the way of the robot
* Floor - friction, slopes
* Weight -  adjust weight weightof robot, add object ontop of robot, change centre of mass of robot

Data Recorder:

* Reduce noise
* Get global coordinates from model_states
* % error
