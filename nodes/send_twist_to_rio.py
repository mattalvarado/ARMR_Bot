#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012

#This node takes a Twist message published to 
#the cmd_vel topic and sends it to the ARMR_Bot cRIO
#It has been tested with the Joy_to_Twist Node in this
#package

import roslib; roslib.load_manifest('ARMR_Bot')
import rospy
from geometry_msgs.msg import Twist
import socket
import struct
import time

class Send_To_cRIO:
	def __init__(self):
        	rospy.init_node('rio_sender')

		#TCP port on cRIO listening for Twist messages
        	self.port = 4567 
        	self.listen_for_rio()
		
        	rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_handler)		
        	rospy.spin()

	def cmd_vel_handler(self, data):
		
 		#Pack the twist message
        	msg = struct.pack('!dddddd', data.linear.x,0,0
		,0,0,data.angular.z)
		try:
			self.sock.send(msg)
			print msg
		except:
			#If there is an error close the connection and
			#Try to reconnect
			self.sock.close()
			self.listen_for_rio()

	def listen_for_rio(self):
		self.sock = socket.socket(socket.AF_INET,
                              socket.SOCK_STREAM)
		self.sock.settimeout(None)
		self.sock.setsockopt(socket.SOL_SOCKET , socket.SO_REUSEADDR, 1)		
		
		try:
			self.sock.connect(('192.168.1.5', self.port))
		except:
			print "Failed to Connect"
			time.sleep(.5)
			self.listen_for_rio()
        	



if __name__ == '__main__':
	twistSender = Send_To_cRIO()
