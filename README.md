# pmb2_stopping_distance

<u><strong>Summary</strong></u>

This script tests the positive linear and angular stopping distance of a robot. 
The current ranges tested for linear are 0.1 m/s to 1.0 m/s.
The current ranges tested for angular are 1.0 m/s to 2.0 m/s.
These can be modified by editing the 'execute_test_scripts' function in stopping_distance.py

The data_recorder node subscribes to 'stop_dist_data' published from pmb2_automation node. 
Data is collected formatted and written to a csv file stored in the 'log' folder

Each scenario has the following logic:
* While robot odometry has not reached desired velocity, continue sending desired velocity
* Once desired velocity is reached and confirmed by the robot odometry, send zero velocity signal
* The stopping distance is then calculated
* This data is written to a new line in csv file
* The process repeats for each scenario

CSV data conists of the following parameters:

* timestamp
* desired_velocity_x
* desired_velocity_z
* stop_distance_x
* stop_distance_z
* stop_time

This includes the desired linear and angular velocities, the linear and angular stopping distances and the time taken to physically stop (since receiving zero velocity command).

Please see sample simulations in the 'video' folder

* short simulation consists of 4 test scenarios - 2 linear: 0.8 m/s and 0.9 m/s and 2 angular: 1.8 m/s and 1.9 m/s. 
* whole simulation consists of 10 linear and 11 angular test scenarios. These include linear velocities from 0.1 to 1.0 m/s and angular velocities from 1.0 to 2.0.

Please see the log folder for the respective CSV output from these test scripts.

* log/short_simulation and video/stop_dist_log_24_02_2021_13_13_24_short_simulation.csv
* log/whole_simulation and video/stop_dist_log_24_02_2021_12_59_31_whole_simulation.csv

<strong>To execute:</strong>

* Install PMB2 package from http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation
* import pmb2_stopping_distance into pmb2_public_ws
* catkin build in pmb2_public_ws
* source ./devel/setup.bash
* roslaunch pmb2_2dnav_gazebo pmb2_mapping.launch public_sim:=true world:=large_corridor
* ./data_recorder.py
* rosrun pmb2_automation stopping_distance.py

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

<strong>Current issues:</strong>

* CMakeLists.txt does not understand when the 'scripts' folder is used.
* Publishers require delay before execution. Need to find out an alternative method to get this working.
