#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012
#This is a hardcoded way to parse flattned Odometry byte strings sent from the cRIO
#This code can be used as a template for how to parse the Odometry byte strings


import roslib; roslib.load_manifest('ARMR_Bot')
import rospy
import time

from twisted.internet.protocol import Protocol, ReconnectingClientFactory
from twisted.internet import reactor
from struct import *
from numpy import *

from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class Odom(Protocol):
	
	def __init__(self):
		self.odom_pub = rospy.Publisher("odom", Odometry)
		self.partial_msg = ''

	def dataReceived(self, data):
		odom = self.decodeMessage(data)    
		 

	def decodeMessage(self, data):
		#Make sure there is a full Odom message
		#If there is not a full message stored data in self.partial_msg
		#If there is more than a full message in self.partial_msg
		#Only remove one message
		self.partial_msg += data
		if len(self.partial_msg) >= 289:
			filt_data = self.partial_msg[0:289]
			self.partial_msg = self.partial_msg[289: len(self.partial_msg)]
			odom = self.unpackOdom(filt_data)
			self.odom_pub.publish(odom)
	
	def unpackOdom(self, data):
		odom_msg = Odometry()
		#Unpack Header
		odom_msg.header = self.unpackOdomHeader(data[0:20])
		#Unpack Child_frame_id
		#Is constant and "base_link"
		odom_msg.child_frame_id = unpack('xxxxccccccccc', data[20:33])[0]
		#Unpack Pose
		pose = self.bytestr_to_array(data[33:89])
		odom_msg.pose.pose.position.x = pose[0]
		odom_msg.pose.pose.position.y = pose[1]
		odom_msg.pose.pose.position.z = pose[2]
		odom_msg.pose.pose.orientation.x = pose[3]
		odom_msg.pose.pose.orientation.y = pose[4]
		odom_msg.pose.pose.orientation.z = pose[5]
		odom_msg.pose.pose.orientation.w = pose[6]
		#Skip Unpack Pose Covariance of Pose
		#The values are all zeros since we did not know how to properly 
		#calculate them. We are not properly sending the correct covarinace
		#maxtrix. Ours is 3x3 where it should be 6x6. To skip the covaraince
		#we will skip 4+ (9*8) = 76 bytes
		#odom_msg.pose.covariance = [float64(0)]*36		

		#Unpack Twist
		twist = self.bytestr_to_array(data[165:213])
		odom_msg.twist.twist.linear.x = twist[0]
		odom_msg.twist.twist.linear.y = twist[1]
		odom_msg.twist.twist.linear.z = twist[2]
		odom_msg.twist.twist.angular.x = twist[3]
		odom_msg.twist.twist.angular.y = twist[4]
		odom_msg.twist.twist.angular.z = twist[5]

		#Skip Unpack Twist Covariance of Pose
		#The values are all zeros since we did not know how to properly 
		#calculate them. We are not properly sending the correct covarinace
		#maxtrix. Ours is 3x3 where it should be 6x6. To skip the covaraince
		#we will skip 4+ (9*8) = 76 bytes

		#odom_msg.twist.covariance = [float64(0)]*36

		return odom_msg
	
	
	def unpackOdomHeader(self, data):
		#Note this is not modular for frame_id's that are not 4 characters
		#This is written for the frame_id = 'odom'
		head = Header()
		unpk = unpack('!IIxxxxccccI', data)
		now  = rospy.Time.now()
		head.stamp.secs = int(now.to_sec())
		head.stamp.nsecs = int(now.to_nsec())
		head.frame_id = unpk[2] + unpk[3] + unpk[4] + unpk[5]
		head.seq = unpk[6]
		return head

	def bytestr_to_array(self,data):
		#This function assumes that the array size is not
		#prepeneded to the data	
		#It also assums that the data bytestring you input is 
		#composed of 64 bit doubles	
		num_arr = []
		
		for i in range(len(data)/8):
			num_arr.append(float(unpack('!d',data[i * 8 : (i * 8) + 8])[0]))
		return num_arr

       

class OdomClientFactory(ReconnectingClientFactory):	
	def __init__(self):
		rospy.init_node('crio_odom_reader')

	def startedConnecting(self, connector):
		print "Started to connect."       

	def buildProtocol(self, addr):
		print "Connected."
		print "Resetting reconnection delay"
		self.resetDelay()
		return Odom()

   	def clientConnectionLost(self, connector, reason):
		print "Lost connection. Reason:", reason
		ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

	def clientConnectionFailed(self, connector, reason):
		print "Connection failed. Reason:", reason
		ReconnectingClientFactory.clientConnectionFailed(self, connector,reason)
		

 

 
#IP address of ARL ARMR Bot cRIO: 192.168.1.5
#port where Odometry data is streaming from : 4568
reactor.connectTCP('192.168.1.5', 4571, OdomClientFactory())
reactor.run()
