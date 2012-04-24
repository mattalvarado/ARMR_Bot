#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012
#Buttons configured for Wireless Xbox 360 controller
#Upper left Joystick if for driving
#B is for E-Stop

import roslib; roslib.load_manifest('ARMR_Bot')
import rospy
from math import exp

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Joy_to_Twist:
	def __init__(self):
		rospy.init_node('joy_to_twist')
		twist_pub = rospy.Publisher('cmd_vel', Twist)
		rospy.Subscriber("joy", Joy, self.callback)
		
		#Do not increase this parameter above 40Hz. 
		#You will see being to see a lag in motor commands
		rate = rospy.Rate(20)

		#Parameters to adjust the ramping for the joystick
		self.fowramp = 1
		self.rotramp = 4

		self.prev_bttn_status = False
		self.estop = False
		self.twistmsg = Twist()
		
		while not rospy.is_shutdown():
			rate.sleep()
			if not self.estop:
				twist_pub.publish(self.twistmsg)
			else:
				twist_pub.publish(Twist())

	def callback(self, data):
		cmd = Twist()
		fow = data.axes[1]
		rot = data.axes[0]

		cmd.linear.x = cmp(fow,0)*(exp(abs(fow)*
		self.fowramp)-1)/(exp(self.fowramp)-1)

		cmd.angular.z = cmp(rot,0)*(exp(abs(rot)*
		self.rotramp)-1)/(exp(self.rotramp)-1)
		
		#The B Button is the Estop
		#prev_bttn_status logic to make sure that you only need to press
		#it once to have it estop. To unestop you need to press it again
		if data.buttons[1] == 0:
			self.twistmsg = cmd
		elif data.buttons[1] !=  self.prev_bttn_status:
			self.estop = not self.estop
		self.prev_bttn_status = data.buttons[1]
	
if __name__ == "__main__": Joy_to_Twist()

