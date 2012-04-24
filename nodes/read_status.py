#!/usr/bin/env python

#Written by Matthew Alvarado Olin College 2012

#This is a hardcoded way to parse flattened Status messages byte strings sent from the cRIO
#This code can be used as a template for how to parse the Status message byte strings

#The status message populated is defined customly (status.msg) in the package where this node exists

import roslib; roslib.load_manifest('ARMR_Bot')
import rospy

from twisted.internet.protocol import Protocol, ReconnectingClientFactory
from twisted.internet import reactor
from struct import *

#This is a custom message
from ARMR_Bot.msg import status



class Status_Class(Protocol):
	
	def __init__(self):
		self.status_pub = rospy.Publisher("status",status)
		self.partial_msg = ''

	def dataReceived(self, data):
		status = self.decodeMessage(data)    
		 

	def decodeMessage(self, data):
		#Make sure there is a full Status message
		#If there is not a full message stored data in self.partial_msg
		#If there is more than a full message in self.partial_msg
		#Only remove one message
		self.partial_msg += data
		if len(self.partial_msg) >= 72:
			filt_data = self.partial_msg[0:72]
			self.partial_msg = self.partial_msg[72: len(self.partial_msg)]
			status = self.unpackStatus(filt_data)
			self.status_pub.publish(status)
	
	def unpackStatus(self, data):
		status_msg = status()
		
		#Unpack Status Message
		msg = self.bytestr_to_array(data[0:72])
		status_msg.five_v = msg[0]
		status_msg.twelve_v = msg[1]
		status_msg.twentyeight_v = msg[2]
		status_msg.fourtyeight_v = msg[3]
		status_msg.temp_a = msg[4]
		status_msg.temp_b = msg[5]
		status_msg.temp_c = msg[6]
		status_msg.temp_d = msg[7]
		status_msg.temp_crio = msg[8]	
		
		return status_msg
	
	

	def bytestr_to_array(self,data):
		#This function assumes that the array size is not
		#prepeneded to the data	
		#It also assums that the data bytestring you input is 
		#composed of 64 bit doubles	
		num_arr = []
		
		for i in range(len(data)/8):
			num_arr.append(float(unpack('!d',data[i * 8 : (i * 8) + 8])[0]))
		return num_arr

       

class StatusClientFactory(ReconnectingClientFactory):	
	def __init__(self):
		rospy.init_node('crio_status_reader')

	def startedConnecting(self, connector):
		print "Started to connect."       

	def buildProtocol(self, addr):
		print "Connected."
		print "Resetting reconnection delay"
		self.resetDelay()
		return Status_Class()

   	def clientConnectionLost(self, connector, reason):
		print "Lost connection. Reason:", reason
		ReconnectingClientFactory.clientConnectionLost(self, connector, reason)

	def clientConnectionFailed(self, connector, reason):
		print "Connection failed. Reason:", reason
		ReconnectingClientFactory.clientConnectionFailed(self, connector,reason)
		

 

 
#IP address of ARL ARMR Bot cRIO: 192.168.1.5
#port where Status mesage is streaming from : 4570
reactor.connectTCP('192.168.1.5', 4570, StatusClientFactory())
reactor.run()
