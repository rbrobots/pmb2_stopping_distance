#!/usr/bin/env python
# Year: 2021
# Author: Ranvir Bhogal
# Description: Stopping distance simulation with TIAgo robot

import math
import rospy
from geometry_msgs.msg import Twist

class Automation():
    def __init__(self):
        print("Initialised")#insert publishers and subscribers below
        self._pub_vel = rospy.Publisher('key_vel',Twist, queue_size=10) # change pub name to my own pub
        self._sub_odom = rospy.Subscriber("/mobile_base_controller/odom",Twist, self.odom_callback)#does this need to be TWIST msg?
        self._distance = 0#change to relevant distance
        self.twist = Twist()

    def odom_callback(self,output):
        print("..in odom_callback function..")
        print("..incoming msg ="+output.data)

    def setPMB2Orientation():#set default orientation in corridoor
        print("in setPMB2Orientation")

    def setVelocity(self, linear, angular):#make parameters dynamic later
        print("In sendVelocity() function")
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def run(self):
        self._linear=0 #initialise var to zero
        self._angular=0
        self._hz = rospy.Rate(1) #MIGHT NEED TO CHANGE THIS

    def publish(self):
        self.setVelocity(0.5,0.0)

        while True:#infinite loop - change this
            self._pub_vel.publish(self.twist)
        print("PUBLISHED")
        #if reached distance x, send zero velocity


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
