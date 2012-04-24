#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012
#This is a hardcoded way to parse flattned IMU byte strings sent from the cRIO
#This code can be used as a template for how to parse the Imu byte strings

#Note when 4 bytes are skipped when indexing the data bytestring that represents
# a I32 corresponding to the size of the string or array. I chose to ignore it
# since I know the IMU server on the ARMR Bots cRIO will transmit Imu messages
# in a predetermined fashion

#This package depends on Twisted, an Event Based python networking library.
#I have implemented a reconnecting client which keeps on trying to reconnect
#to the IMU server on the cRIO. 

import roslib; roslib.load_manifest('ARMR_Bot')
import rospy
import time

from twisted.internet.protocol import Protocol, ReconnectingClientFactory
from twisted.internet import reactor
from struct import *
from numpy import *

from sensor_msgs.msg import Imu
from std_msgs.msg import Header



class IMU(Protocol):
	
	def __init__(self):
		self.imu_pub = rospy.Publisher("imu",Imu)
		self.partial_msg = ''

	def dataReceived(self, data):
		imu = self.decodeMessage(data)    
		

	def decodeMessage(self, data):
		#Make sure there is a full IMU message
		#If there is not a full message stored data in self.partial_msg
		#If there is more than a full message in self.partial_msg
		#Only remove one message
		self.partial_msg += data
		if len(self.partial_msg) >= 327:
			filt_data = self.partial_msg[0:327]
			self.partial_msg = self.partial_msg[327:len(self.partial_msg)]
			imu = self.unpackIMU(filt_data)
			self.imu_pub.publish(imu) 
	
	def unpackIMU(self, data):
		imu_msg = Imu()
		#Unpack Header
		imu_msg.header = self.unpackIMUHeader(data[0:19])
		#Unpack Orientation Message
		quat = self.bytestr_to_array(data[19:19 + (4*8)])
		imu_msg.orientation.x = quat[0]
		imu_msg.orientation.y = quat[1]
		imu_msg.orientation.z = quat[2]
		imu_msg.orientation.w = quat[3]
		#Unpack Orientation Covariance
		imu_msg.orientation_covariance = list(self.bytestr_to_array(data[55:(55 + (9*8))]))
		#Unpack Angular Velocity
		ang = self.bytestr_to_array(data[127: 127 + (3*8)])
		imu_msg.angular_velocity.x = ang[0]
		imu_msg.angular_velocity.y = ang[1]
		imu_msg.angular_velocity.z = ang[2]
		#Unpack Angular Velocity Covariance
		imu_msg.angular_velocity_covariance = list(self.bytestr_to_array(data[155:(155 + (9*8))]))
		#Unpack Linear Acceleration
		lin = self.bytestr_to_array(data[227: 227 + (3*8)])
		imu_msg.linear_acceleration.x = lin[0]
		imu_msg.linear_acceleration.y = lin[1]
		imu_msg.linear_acceleration.z = lin[2]
		#Unpack Linear Acceleration Covariance
		imu_msg.linear_acceleration_covariance = list(self.bytestr_to_array(data[255:(255 + (9*8))]))
		return imu_msg	
	
	
	def unpackIMUHeader(self, data):
		#Note this is not modular for frame_id's that are not 3 characters
		#This is written for the frame_id = 'imu'
		head = Header()
		unpk = unpack('!IIxxxxcccI', data)
		now  = rospy.Time.now()
		head.stamp.secs = int(now.to_sec())
		head.stamp.nsecs = int(now.to_nsec())
		head.frame_id = unpk[2] + unpk[3] + unpk[4]
		head.seq = unpk[5]
		return head

	def bytestr_to_array(self,data):
		#This function assumes that the array size is not
		#prepeneded to the data		
		num_arr = []
		
		for i in range(len(data)/8):
			num_arr.append(float64(unpack('!d',data[i * 8 : (i * 8) + 8])[0]))
		return num_arr

       

class IMUClientFactory(ReconnectingClientFactory):	
	def __init__(self):
		rospy.init_node('crio_Imu_reader')

	def startedConnecting(self, connector):
		print "Started to connect."       

	def buildProtocol(self, addr):
		print "Connected."
		print "Resetting reconnection delay"
		self.resetDelay()
		return IMU()

   	def clientConnectionLost(self, connector, reason):
		print "Lost connection. Reason:", reason
		ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

	def clientConnectionFailed(self, connector, reason):
		print "Connection failed. Reason:", reason
		ReconnectingClientFactory.clientConnectionFailed(self, connector,reason)
		

 

 
#IP address of ARL ARMR Bot cRIO: 192.168.1.5
#port where IMU data is streaming from : 4569
reactor.connectTCP('192.168.1.5', 4569, IMUClientFactory())
reactor.run()
 
	
	


		
		


