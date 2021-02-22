#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Stopping distance simulation with TIAgo robot

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Automation():
    def __init__(self):
        print("Initialised")#insert publishers and subscribers below
        self._pub_vel = rospy.Publisher('key_vel',Twist, queue_size=10) # change pub name to my own pub
        self._sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry, self.odom_callback)#does this need to be TWIST msg?
        self._distance = 0#change to relevant distance NOT CUR USED
        self.twist = Twist()#target velocity
        self.robot_odometry = Odometry()

    def odom_callback(self,robot):#store odom info in robot_odometry
        print("..in odom_callback function..")
        self.robot_odometry.pose.pose.position.x=robot.pose.pose.position.x
        self.robot_odometry.pose.pose.orientation.z=robot.pose.pose.orientation.z
        self.robot_odometry.twist.twist.linear.x = robot.twist.twist.linear.x
        self.robot_odometry.twist.twist.angular.z = robot.twist.twist.angular.z
        print()

    def setPMB2Orientation():#set default orientation in corridoor
        print("in setPMB2Orientation")

    def setVelocity(self, linear, angular):#make parameters dynamic later
        print("In sendVelocity() function")
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def run(self):
        #self._linear=0 #initialise var to zero
        #self._angular=0
        self._hz = rospy.Rate(2) #MIGHT NEED TO CHANGE THIS

    def publish(self):
        self.setVelocity(0.5,0.0)
        print(round(self.robot_odometry.twist.twist.linear.x,2))
        while round(self.robot_odometry.twist.twist.linear.x,2)!=self.twist.linear.x:#while robot local velocity <= commanded velocity
            self._pub_vel.publish(self.twist)#send velocity signal
            print(round(self.robot_odometry.twist.twist.linear.x,2))
            print(self.twist.linear.x)
            if(round(self.robot_odometry.twist.twist.linear.x,2) == self.twist.linear.x):
                print("WE HIT DESIRED VELOCITY")
        else:
            print("WE HAVE HIT DESIRED VELOCITY")
        #print("..sending vel command..")
        print("..ROBOT STOPPED..")
        #if reached distance x, send zero velocity
        #start counting time until full stop detected in odometry

if __name__ == '__main__':
    try:
        rospy.init_node('pmb2_automation')

        aut = Automation()#instantiate class
        aut.run()
        aut.publish()

        #rate = rospy.Rate(1)#publish at 1 Hz/s -- should I change the rate?

        #while not rospy.is_shutdown():
        #    aut.setPMB2Orientation()
        #    aut.sendVelocity()
            
    except rospy.ROSInterruptException:
        pass
