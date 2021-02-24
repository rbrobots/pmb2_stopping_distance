#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Stopping distance simulation with mobile robot

import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from pmb2_automation.msg import Data
from std_msgs.msg import String, Int8, Float32, Time, Int32 
#from gazebo.srv import *

class Automation():
    def __init__(self):
        print("Initialised")#insert publishers and subscribers below
        self._pub_model_state = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)
        self._pub_vel = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist, queue_size=10)
        self._pub_data = rospy.Publisher('stop_dist_data',Data,queue_size=10)
        #self._pub_script_status = rospy.Publisher('script_status',String,queue_size=10)

        self._sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry, self.odom_callback)
        
        #self._sub_model_states = rospy.Subscriber('/gazebo/model_states',ModelStates, self.model_states_callback)
        #better to use service instead?

        self.model_state = ModelState()#for setting model state at each iteration of test scenario
        self.target_velocity = Twist()#target velocity
        self.robot_odometry = Odometry()#robot odometry values
        self.timer1 =0
        self.timer2 =0
        self.time_elapsed =0
        self.movement=True 
        self.stop_dist_actual = Pose()#stopping distance actual
        self.stop_dist_1 = Pose()#when signal is sent for 0 velocity
        self.stop_dist_2 = Pose()#when 0 velocity detected in wheels
        self.t_vel_x =0.0
        self.t_vel_z =0.0
        self.sd1 = False#flags for stop distance
        self.sd2 = False

        self.data = Data()

        self.script_execution = False

    def reset(self):#CHECK THIS
        self.model_state = ModelState()#for setting model state at each iteration of test scenario
        self.target_velocity = Twist()#target velocity
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

    def odom_callback(self,robot):#store odom info in robot_odometry
        #print("..in odom_callback function..")
        self.robot_odometry.pose.pose.position.x=robot.pose.pose.position.x
        self.robot_odometry.pose.pose.orientation.z=robot.pose.pose.orientation.z
        self.robot_odometry.twist.twist.linear.x = robot.twist.twist.linear.x
        self.robot_odometry.twist.twist.angular.z = robot.twist.twist.angular.z


    #def get_model_state(self,model_name,entity_name):#UPDATE TO SERVICE CALL. FUNCTION NOT USED
        #rospy.wait_for_service('/gazebo/get_model_state')
        #try: 
            #gms = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
            #res = gms(model_name,entity_name)
            #return res
        #except rospy.ServiceException, e:
            #print("service call error")
        #if(s=="p_x"):
            #return -1#self.model_state.pose.position.x
        #elif(s=="p_y"):
            #return -1#self.model_state.pose.position.y
        #elif(s=="o_z"):
            #return -1#self.model_state.pose.orientation.z

    def set_model_state(self,p_x,p_y,p_z,o_x,o_y,o_z,o_w):# add rest of arguments
        self.model_state.model_name = "pmb2"#this should be dynamic later
        self.model_state.pose.position.x = p_x
        self.model_state.pose.position.y = p_y
        self.model_state.pose.position.z = p_z
        self.model_state.pose.orientation.x= o_x
        self.model_state.pose.orientation.y= o_y
        self.model_state.pose.orientation.z= o_z#perp to wall in corridoor
        self.model_state.pose.orientation.w =o_w
        #all twist values hardcoded to zero for this case
        self.model_state.twist.linear.x=0.0
        self.model_state.twist.linear.y=0.0
        self.model_state.twist.linear.z=0.0
        self.model_state.twist.angular.x=0.0
        self.model_state.twist.angular.y=0.0
        self.model_state.twist.angular.z=0.0
        self.model_state.reference_frame = "world"

    def pub_model_state(self):#set default pmb2 pose in corridoor
        time.sleep(2)
        self.set_model_state(self.model_state.pose.position.x,self.model_state.pose.position.y,self.model_state.pose.position.z,
            self.model_state.pose.orientation.x,self.model_state.pose.orientation.y,self.model_state.pose.orientation.z,
            self.model_state.pose.orientation.w)
        #self.set_model_state(0.0,0.0,0.0,0.0,0.0,1.57,1.0)
        self._pub_model_state.publish(self.model_state)
        time.sleep(2)
        #print("model state initialised with pmb2 orientation.z=",self.model_state.pose.orientation.z)

    def set_target_velocity(self, linear, angular):#this is just for storage. Not published yet
        #print("In sendVelocity() function")
        self.target_velocity.linear.x = linear
        self.target_velocity.angular.z = angular

    def pub_target_velocity(self):
        self._pub_vel.publish(self.target_velocity)

    def calc_stop_dist(self,n):
        if(n==1):#initial zero velocity command sent
            self.stop_dist_1.position.x = self.robot_odometry.pose.pose.position.x
            self.stop_dist_1.orientation.z = self.robot_odometry.pose.pose.orientation.z
            #print("stop_dist_1 x=",self.stop_dist_1.position.x," stop_dist_1 z=",self.stop_dist_1.orientation.z)
            self.sd1 = True
        elif(n==2):#physical robot stops with zero velocity
            self.stop_dist_2.position.x = self.robot_odometry.pose.pose.position.x
            self.stop_dist_2.orientation.z = self.robot_odometry.pose.pose.orientation.z
            #print("stop_dist_2 x=",self.stop_dist_2.position.x," stop_dist_2 z=",self.stop_dist_2.orientation.z)
            self.sd2 = True
        elif(n==3):#calculate absolute? difference between 2 and 1;
            self.stop_dist_actual.position.x = abs(self.stop_dist_2.position.x - self.stop_dist_1.position.x)
            self.stop_dist_actual.orientation.z = abs(self.stop_dist_2.orientation.z - self.stop_dist_1.orientation.z)
            #print("stop_act_x=",self.stop_dist_actual.position.x," stop_act_z=",self.stop_dist_actual.orientation.z)


    def calc_elapsed_time(self,n):
        if(n==1):
            self.timer1 = time.time()
        elif(n==2):
            self.timer2 = time.time()
        elif(n==3):
            self.time_elapsed = self.timer2-self.timer1

    def execute_test_scripts(self):#this script will iterate through a number of test scenarios
        #LINEAR from 0.1 to 1.0
        for x in np.arange(0.10, 1.10, 0.10):#iterate all linear speeds (0.1-1.0) for x within speed restrictions.
            print("..running test case cmd vel x=",round(x,1))
            self.execute_single_test_script(x,0.0,"linear")
            print("..test case cmd vel x=",round(x,1))

        #ROTATION from 1.0 to 2.0
        for z in np.arange(1.0, 2.10, 0.10):#Update parameters to be user-defined
            print("..running test case cmd vel z=",round(z,1))
            self.execute_single_test_script(0.0,z,"angular")
            print("..test case cmd vel z=",round(z,1))

    def execute_single_test_script(self, x, z, direction):
        self.reset()
        self.set_model_state(0.0,0.0,0.00000000,0.0,0.0,1.0,1.0)#initial model state always the same
        self.pub_model_state()
        #initialisations done

        self.set_target_velocity(x,z)
        self.t_vel_x=x
        self.t_vel_z=z
        self.pub_target_velocity()

        time.sleep(2)

        if(direction=="linear"):

            while True:
                if(round(self.robot_odometry.twist.twist.linear.x,2) >= round(self.target_velocity.linear.x,1)):#if robot has reached target velocity
                    #print("(IN 1) 2 dp odom=",round(self.robot_odometry.twist.twist.linear.x,2))
                    self.set_target_velocity(0.0,0.0)
                    self.pub_target_velocity()
                    if(self.sd1==False):
                        self.calc_stop_dist(1)#starting calculating stop distance
                    self.calc_elapsed_time(1)#start calculating time to real stop
                    self.movement=False
                else:
                    self.pub_target_velocity()#if not at target velocity, send velocity command to robot for this test case

                if(self.robot_odometry.twist.twist.linear.x<=0 and self.movement==False):#if odometry shows robot has stopped
                    if(self.sd2==False):
                        self.calc_stop_dist(2)#stop calculating stop distance
                    self.calc_elapsed_time(2)#stop caclulating stop distance time
                    self.calc_elapsed_time(3)#calculate time taken to stop robot after receiving zero velocity
                    self.calc_stop_dist(3)#calculate actual stop distance
                    #print("(IN 3) 2 dp odom=",round(self.robot_odometry.twist.twist.linear.x,2))
                    break
        if(direction=="angular"):
            while True:
                if(round(self.robot_odometry.twist.twist.angular.z,2) >= round(self.target_velocity.angular.z,1)):#if robot has reached target velocity
                    #print("(IN 1) 2 dp odom=",round(self.robot_odometry.twist.twist.linear.x,2))
                    self.set_target_velocity(0.0,0.0)
                    self.pub_target_velocity()
                    if(self.sd1==False):
                        self.calc_stop_dist(1)#starting calculating stop distance
                    self.calc_elapsed_time(1)#start calculating time to real stop
                    self.movement=False
                else:
                    self.pub_target_velocity()#if not at target velocity, send velocity command to robot for this test case

                if(self.robot_odometry.twist.twist.angular.z<=0 and self.movement==False):#if odometry shows robot has stopped
                    if(self.sd2==False):
                        self.calc_stop_dist(2)#stop calculating stop distance
                    self.calc_elapsed_time(2)#stop caclulating stop distance time
                    self.calc_elapsed_time(3)#calculate time taken to stop robot after receiving zero velocity
                    self.calc_stop_dist(3)#calculate actual stop distance
                    #print("(IN 3) 2 dp odom=",round(self.robot_odometry.twist.twist.linear.x,2))
                    break
        self.pub_data()#publish collected data to topic for results analysis



    def pub_data(self):#this function publishes data for output
        #self.data.desired_velocity.linear.x = self.target_velocity.linear.x
        self.data.desired_velocity_x = self.t_vel_x
        self.data.desired_velocity_z = self.t_vel_z
        self.data.initial_pose_x=0.0##initial pose is always 0
        self.data.initial_pose_y=0.0
        self.data.initial_twist_z=1.0#make dynamic!

        self.data.stop_distance_x=self.stop_dist_actual.position.x#doesn't like this
        self.data.stop_distance_z=self.stop_dist_actual.orientation.z#doesn't like this
        self.data.stop_time=self.time_elapsed


        #THE FOLLOWING ARE NOT YET USED.
        self.data.final_pose_x=0#self.get_model_state("p_x")
        self.data.final_pose_y=0#self.get_model_state("p_y")
        self.data.final_twist_z=0#self.get_model_state("o_z")

        print("velocity.x=",self.t_vel_x)
        print("stop_distance.x=",self.data.stop_distance_x)
        print("velocity.z=",self.t_vel_z)
        print("stop_distance.z=",self.data.stop_distance_z)
        #self.data.dist_error=0#TO ADD
        #output laser data
        time.sleep(2)
        self._pub_data.publish(self.data)
        time.sleep(2)

    def publish(self):
        #self.set_target_velocity(5,0.0)
        while True:#while robot local velocity <= commanded velocity

            if(round(self.robot_odometry.twist.twist.linear.x,2) >= self.target_velocity.linear.x):#we only check linear velocity
                #print("WE HIT DESIRED VELOCITY")
                self.set_target_velocity(0.0,0.0)#this doesn't do anything
                
                if(self.sd1==False):#calc initial stop dist
                #get pose from robot
                    self.calc_stop_dist(1)

                self.timer1 = time.time()
                #print("Elapsed Time:")
                self.movement=False#this might slow down readings
            else:
                self._pub_vel.publish(self.target_velocity)


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
        self.pub_data()


if __name__ == '__main__':
    try:
        rospy.init_node('pmb2_automation')

        aut = Automation()#instantiate class

        rate = rospy.Rate(1)#publish at 1 Hz/s -- should I change the rate?

        aut.execute_test_scripts()
        
        rospy.loginfo("script execution complete")

            
    except rospy.ROSInterruptException:
        pass
