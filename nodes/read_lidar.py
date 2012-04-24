#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012

#This is a hardcoded way to parse flattened LIDAR byte strings sent from the cRIO
#This code can be used as a template for how to parse the LIDAR byte strings

#Note when 4 bytes are skipped when indexing the data bytestring that represents
# a I32 corresponding to the size of the string or array. I chose to ignore it
# since I know the LIDAR server on the ARMR_Bot's cRIO will transmit LaserScan messages
# in a predetermined fashion

#This package depends on Twisted, an Event Based python networking library.
#I have implemented a reconnecting client which keeps on trying to reconnect
#to the cRIO LIDAR server on the cRIO.


import roslib; roslib.load_manifest('ARMR_Bot')
import rospy
import time

from twisted.internet.protocol import Protocol, ReconnectingClientFactory
from twisted.internet import reactor
from struct import *



from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header



class LIDAR(Protocol):
	
	def __init__(self):		
		self.lidar_pub = rospy.Publisher("scan",LaserScan)
		self.partial_msg = ''

	def dataReceived(self, data):
		lidar = self.decodeMessage(data)    
		 

	def decodeMessage(self, data):
		#Make sure there is a full LIDAR message
		#If there is not a full message stored data in self.partial_msg
		#If there is more than a full message in self.partial_msg
		#Only remove one message
		self.partial_msg += data
		if len(self.partial_msg) >= 8733:
			filt_data = self.partial_msg[0:8733]
			self.partial_msg = self.partial_msg[8733: len(self.partial_msg)]
			lidar = self.unpackLIDAR(filt_data)
			self.lidar_pub.publish(lidar)
	
	def unpackLIDAR(self, data):
		#This is a hardcoded way to parse LIDAR data from the RIO.
		#Please use this as a framework to 
		
		lidar_msg = LaserScan()
		#Unpack Header
		lidar_msg.header = self.unpackLIDARHeader(data[0:21])
		#Unpack Sensor Params
		param = self.bytestr_to_array(data[21:21 + (7*8)])
		lidar_msg.angle_min = param[0]
		lidar_msg.angle_max = param[1]
		lidar_msg.angle_increment = param[2]
		lidar_msg.time_increment = param[3]
		lidar_msg.scan_time = param[4]
		lidar_msg.range_min = param[5]
		lidar_msg.range_max = param[6]
		#Unpack Range Data
		lidar_msg.ranges = list(self.bytestr_to_array(data[81: 81 + (1081*8)]))
		#There is no intensity data so there is no need to parse it
		return lidar_msg
	
	
	def unpackLIDARHeader(self, data):
		#Note this is not modular for frame_id's that are not 5 characters
		#This is written for the frame_id = 'laser'
		head = Header()
		unpk = unpack('!IIxxxxcccccI', data)
		now  = rospy.Time.now()
		head.stamp.secs = int(now.to_sec())
		head.stamp.nsecs = int(now.to_nsec())
		head.frame_id = unpk[2] + unpk[3] + unpk[4] + unpk[5] + unpk[6]
		head.seq = unpk[7]
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

       

class LIDARClientFactory(ReconnectingClientFactory):	
	def __init__(self):
		rospy.init_node('crio_Lidar_reader')

	def startedConnecting(self, connector):
		print "Started to connect."       

	def buildProtocol(self, addr):
		print "Connected."
		print "Resetting reconnection delay"
		self.resetDelay()
		return LIDAR()

   	def clientConnectionLost(self, connector, reason):
		print "Lost connection. Reason:", reason
		ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

	def clientConnectionFailed(self, connector, reason):
		print "Connection failed. Reason:", reason
		ReconnectingClientFactory.clientConnectionFailed(self, connector,reason)
		

 

 
#IP address of ARL ARMR Bot cRIO: 192.168.1.5
#port where LIDAR data is streaming from : 4568
reactor.connectTCP('192.168.1.5', 4568, LIDARClientFactory())
reactor.run()
