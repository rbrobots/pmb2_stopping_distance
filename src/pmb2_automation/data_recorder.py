#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Data class to collect data from stopping_distance.py automated test script

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry

class DataRecorder():
	def __init__(self):
		self.unique_id=0#unique id for this test
		self.desired_velocity=Twist()#desired velocity

		self.initial_pose=Pose()#inital pose
		self.final_pose=Pose()#final pose

		self.stop_dist=0#stopping distance
		self.robot_zero_velocity_time_taken=0##time it took to actually reach zero velocity

		self.extra_dist_travelled=0#% extra distance travelled (% error)

		#include global coords

		#self.current_linear_velocity=0#current velocity?? **

		#self.current_angular_velocity=0#current velocity?? **
		#time taken to reach desired velocity

		#self.desired_angular_velocity=Twist()#desired velocity



		