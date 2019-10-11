
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	ROSNodeManager.py 
# Description:	ROSNodeManager class implementation	(Headless ANDS)
#				ROSNodeManager class creates a two ROS nodes, one for the voltage, one for the
#				proximity, and then continually publishes and subscribes to the data coming into 
# 				the ground unit   

import geometry_msgs.msg
import std_msgs.msg
import rospy
from visualization_msgs.msg import Marker
from time import sleep

class ROSNodeManager:
	
	__VOLTAGE_CHAR = 'v'
	__PROXIMITY_CHAR_START = 'a'
	__PROXIMITY_CHAR_END = 'n'

	# voltage and proximity topic publisher nodes
	__voltagePublisher = None
	__proximityPublisher = None

	# Name:		Default constructor
	# Description:	ROSNodeManager class default constructor which initializes the node and then creates a number
	# 				of other nodes, both publishers and subscribers, by calling the various initializer methods
	# Parameters:	None
	# Return:		None
	def __init__(self):

		rospy.init_node('headlessGroundUnitNode')
		self.initializeVoltageNode()
		self.initializeProximityNode()

	# Name:		initializeVoltageNode
	# Description:	ROSNodeManager class method which initializes the voltage publisher node which 
	# 				continually publishes the voltage value  
	# Parameters:	None
	# Return:		None
	def initializeVoltageNode(self):

		self.__voltagePublisher = rospy.Publisher('headlessVoltagePublish', std_msgs.msg.String, queue_size=1)

	# Name:		initializeVoltageNode
	# Description:	ROSNodeManager class method which initializes the proximity publisher node which 
	# 				continually publishes the proximity values  
	# Parameters:	None
	# Return:		None
	def initializeProximityNode(self):

		# for publishing the proximity as a string
		#self.__proximityPublisher = rospy.Publisher('proximityPublish', std_msgs.msg.String, queue_size=1)

		# for publishing the proximity as a Marker object
		self.__proximityPublisher = rospy.Publisher('headlessProximityPublish', Marker, queue_size=10)		

	# Name:		convertSensorDesc
	# Description:	ROSNodeManager class method which takes in a sensor identifier as a char, converts
	# 				it to an ordinal and appends it to the end of the quaternion label, which is returned
	# 				as a string so that the correct sensor on-board the drone can be referenced for the 
	# 				quaternion launch file builder
	# Parameters:	Takes in an identifier to convert to an ordinal
	# Return:		Returns a string which is a quaternion label that can be used when relating the current
	# 				proximity reading with the corresponding sensor on-board the drone
	def convertSensorDesc(self, identifier):

		return "/quat/tof_tf_" + str(ord(identifier) - ord('a') + 1)

	# Name:		publishSensorValue
	# Description:	ROSNodeManager class method which takes in a value, checks whether it is a 
	# 				voltage or a proximity reading, and publishes the value either as a voltage
	# 				topic or a proximity topic, if it is a proximity reading then it is first converted
	# 				into a marker message which is tied to the quaternion of the corresponding sensor 
	# 				on-board the drone
	# Parameters:	Takes in a sensor value, either a voltage or a proximity, and publishes it to the
	# 				appropriate topic
	# Return:		None
	def publishSensorValue(self, sensorValue):

		if sensorValue != None and len(sensorValue) > 1:
			if sensorValue[0] == self.__VOLTAGE_CHAR:
				self.__voltagePublisher.publish(str(sensorValue))
				
			elif sensorValue[0] >= self.__PROXIMITY_CHAR_START and sensorValue[0] <= self.__PROXIMITY_CHAR_END:
				# for publishing the proximity as a Marker object
				val = ord( sensorValue[0] ) - ord('a') + 1
				marker = Marker()
				marker.header.frame_id = self.convertSensorDesc( sensorValue[0] )
				
				marker.header.stamp = rospy.Time()
				marker.ns = "tof_system"
				marker.id = ord( sensorValue[0] )
				marker.type = 0
				marker.action = 0
				marker.pose.position.x = 0
				marker.pose.position.y = 0
				marker.pose.position.z = 0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0
				marker.scale.x = int(sensorValue[1]) * 0.001
				
				marker.scale.y = 0.1
				marker.scale.z = 0.1
				marker.color.a = 1.0
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.0
		
				self.__proximityPublisher.publish( marker )
				
				# for publishing the proximity as a string
				#self.__proximityPublisher.publish(str(sensorValue))


	