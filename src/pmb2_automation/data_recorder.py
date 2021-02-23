#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Data class to collect data from stopping_distance.py automated test script

import rospy
import math
import time
import csv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from pmb2_automation.msg import Data
from std_msgs.msg import String, Int8, Float32, Time, Int32 

class DataRecorder():
	def __init__(self):

		self._sub_stop_dist = rospy.Subscriber('stop_dist_data',Data, self.stop_dist_callback)
		self.data=Data()
		rate= rospy.Rate(1)

		self.csvtime = rospy.get_time()

		csv_name='stop_dist_log'+str(self.csvtime)+'.csv'

		while True:
			rate.sleep()
			self.timestamp = rospy.get_time()
			state_record = str(self.timestamp) + ',' + \
			str(self.data.desired_velocity_x) + ',' + \
			str(self.data.desired_velocity_z) + ',' + \
			str(self.data.initial_pose_x) + ',' + \
			str(self.data.initial_pose_y) + ',' + \
			str(self.data.initial_twist_z) + ',' + \
			str(self.data.final_pose_x) + ',' + \
			str(self.data.final_pose_y) + ',' + \
			str(self.data.final_twist_z) + ',' + \
			str(self.data.stop_distance) + ',' + \
			str(self.data.stop_time)+ ',' + \
			str(self.data.dist_error)

			with open(csv_name, 'a') as logfile:
				logfile.write(state_record + "\n")
				logfile.flush()



	def stop_dist_callback(self,output):
		self.data.desired_velocity_x= output.desired_velocity_x
		self.data.desired_velocity_z= output.desired_velocity_z
		self.data.initial_pose_x=output.initial_pose_x
		self.data.initial_pose_y=output.initial_pose_y
		self.data.initial_twist_z=output.initial_twist_z
		self.data.final_pose_x=output.final_pose_x
		self.data.final_pose_y=output.final_pose_y
		self.data.final_twist_z=output.final_twist_z
		self.data.stop_distance=output.stop_distance#doesn't like this
		self.data.stop_time=output.stop_time
		self.data.dist_error=output.dist_error

if __name__ == '__main__':
	rospy.init_node('data_recorder', disable_signals=True)
	data_recorder = DataRecorder()
	rospy.spin()

		