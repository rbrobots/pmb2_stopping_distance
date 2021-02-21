#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal

import math
import rospy
from geometry_msgs.msg import Twist

class Automation():
    def __init__(self):
        print("Initialised")#insert publishers and subscribers below
        self._pub_vel = rospy.Publisher('key_vel',Twist, queue_size=10) # change pub name to my own pub

#        self.rospy.Subscriber("robot_pose") #NEED TO GET FEEDBACK OF ROBOT POSITION - TO UPDATE * 
# add velocity boundaries


    def setPMB2Orientation():#set default orientation in corridoor
        print("in setPMB2Orientation")

    def sendVelocity(self):#make parameters dynamic later
        print("In sendVelocity() function")

    def run(self):
        self._linear=0 #initialise var to zero
        self._angular=0
        self._hz = rospy.Rate(1) #MIGHT NEED TO CHANGE THIS


    def publish(self):
        twist=Twist()
        twist.linear.x = 0.5
        twist.angular.z = 1
        while True:
            self._pub_vel.publish(twist)
        print("PUBLISHED")


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
