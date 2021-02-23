#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Stopping distance simulation with TIAgo robot

import math
import rospy
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry

class Automation():
    def __init__(self):
        print("Initialised")#insert publishers and subscribers below
        self._pub_model_state = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
        self._pub_vel = rospy.Publisher('key_vel',Twist, queue_size=10) # change pub name to my own pub
        self._pub_data = rospy.Publisher('data_pub',)
        self._sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry, self.odom_callback)

        self.model_state = ModelState()#for setting model state at each iteration of test scenario
        self.twist = Twist()#target velocity
        self.robot_odometry = Odometry()#robot odometry values
        self.timer1 =0
        self.timer2 =0
        self.time_elapsed =0
        self.movement=True # change this its messy
        self.stop_dist_actual = Pose()#stopping distance actual
        self.stop_dist_1 = Pose()#when signal is sent for 0 velocity
        self.stop_dist_2 = Pose()#when 0 velocity detected in wheels
        self.sd1 = False#flags for stop distance
        self.sd2 = False

        #self.initModelState()#set initial pose of robot

    def odom_callback(self,robot):#store odom info in robot_odometry
        #print("..in odom_callback function..")
        self.robot_odometry.pose.pose.position.x=robot.pose.pose.position.x
        self.robot_odometry.pose.pose.orientation.z=robot.pose.pose.orientation.z
        self.robot_odometry.twist.twist.linear.x = robot.twist.twist.linear.x
        self.robot_odometry.twist.twist.angular.z = robot.twist.twist.angular.z

    def setModelState(self,p_x,p_y,p_z,o_x,o_y,o_z,o_w):# add rest of arguments
        self.model_state.model_name = "pmb2"
        self.model_state.pose.position.x = p_x
        self.model_state.pose.position.y = p_y
        self.model_state.pose.position.z = p_z
        self.model_state.pose.orientation.x= o_x
        self.model_state.pose.orientation.y= o_y
        self.model_state.pose.orientation.z= o_z#perp to wall in corridoor
        self.model_state.pose.orientation.w =o_w
        self.model_state.twist.linear.x=0.0
        self.model_state.twist.linear.y=0.0
        self.model_state.twist.linear.z=0.0
        self.model_state.twist.angular.x=0.0
        self.model_state.twist.angular.y=0.0
        self.model_state.twist.angular.z=0.0
        self.model_state.reference_frame = "world"

    def initModelState(self):#set default pmb2 pose in corridoor
        time.sleep(2)
        self.setModelState(0.0,0.0,0.0,0.0,0.0,1.57,1.0)
        self._pub_model_state.publish(self.model_state)
        time.sleep(2)
        print("model state initialised ",self.model_state.pose.orientation.z)

    def setVelocity(self, linear, angular):#make parameters dynamic later
        #print("In sendVelocity() function")
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def run(self):
        print("run")
        #self._hz = rospy.Rate(1) #MIGHT NEED TO CHANGE THIS

    def calc_stop_dist(self,n):
        if(n==1):#initial zero velocity command sent
            self.stop_dist_1.position.x = self.robot_odometry.pose.pose.position.x
            self.stop_dist_1.orientation.z = self.robot_odometry.pose.pose.orientation.z
            print("stop_dist_1 x=",self.stop_dist_1.position.x," stop_dist_1 z=",self.stop_dist_1.orientation.z)
            self.sd1 = True
        elif(n==2):#physical robot stops with zero velocity
            self.stop_dist_2.position.x = self.robot_odometry.pose.pose.position.x
            self.stop_dist_2.orientation.z = self.robot_odometry.pose.pose.orientation.z
            print("stop_dist_2 x=",self.stop_dist_2.position.x," stop_dist_2 z=",self.stop_dist_2.orientation.z)
            self.sd2 = True
        elif(n==3):#calculate absolute? difference between 2 and 1;
            self.stop_dist_actual.position.x = abs(self.stop_dist_2.position.x - self.stop_dist_1.position.x)
            self.stop_dist_actual.orientation.z = abs(self.stop_dist_2.orientation.z - self.stop_dist_1.orientation.z)
            print("stop_act_x=",self.stop_dist_actual.position.x," stop_act_z=",self.stop_dist_actual.orientation.z)

    def testScript():
        self.setVelocity(0.1,0.0) # 2d for loop iterate all speeds

    def publishData():#get all data and publish


    def publish(self):
        self.setVelocity(0.7,0.0)
        while True:#while robot local velocity <= commanded velocity
            if(round(self.robot_odometry.twist.twist.linear.x,2) >= self.twist.linear.x):
                #print("WE HIT DESIRED VELOCITY")
                self.setVelocity(0.0,0.0)#is this needed because the robot stopped receiving velocity commands?
                
                if(self.sd1==False):#calc initial stop dist
                #get pose from robot
                    self.calc_stop_dist(1)

                self.timer1 = time.time()
                #print("Elapsed Time:")
                self.movement=False#this might slow down readings
            else:
                self._pub_vel.publish(self.twist)
            if(self.robot_odometry.twist.twist.linear.x<=0 and self.movement==False):
                print("ROBOT HAS STOPPED 1")

                #get pose from robot
                if(self.sd2==False):
                    self.calc_stop_dist(2)

                self.timer2 = time.time()
                self.time_elapsed = self.timer2-self.timer1
                
                self.calc_stop_dist(3)#print actual stop distance

                print("Elapsed Time:")
                print(self.time_elapsed)
                print("Stop Distance x:")
                print(self.stop_dist_actual.position.x)
                break

        else:
            print("..ROBOT STOPPED 2..")


if __name__ == '__main__':
    try:
        rospy.init_node('pmb2_automation')

        aut = Automation()#instantiate class

        rate = rospy.Rate(1)#publish at 1 Hz/s -- should I change the rate?
        time.sleep(2)
        aut.initModelState()
        time.sleep(2)

        aut.run()
        aut.publish()

        #while not rospy.is_shutdown():
            #rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
