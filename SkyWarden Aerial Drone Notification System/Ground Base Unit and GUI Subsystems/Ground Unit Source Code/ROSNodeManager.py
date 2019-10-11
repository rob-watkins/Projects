
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	ROSNodeManager.py 
# Description:	ROSNodeManager class implementation	
#				ROSNodeManager class creates a number of ROS nodes, one for the voltage, one for
#				the proximity, four for generic alerts tied to toggle switches on the ground unit,
#				and eight for generic alerts tied to LEDs on the ground unit, and then continually
#				publishes and subscribes to the data coming into, and being generated, on the 
#				ground unit   

import geometry_msgs.msg
import std_msgs.msg
import rospy
from visualization_msgs.msg import Marker
from time import sleep

class ROSNodeManager:

	__HEX_SIZE = 8
	__ZERO = 0
	__HEX_ZERO = 0x00
	__HEX_ONE = 0x01
	
	__VOLTAGE_CHAR = 'v'
	__PROXIMITY_CHAR_START = 'a'
	__PROXIMITY_CHAR_END = 'n'

	# voltage and proximity topic publisher nodes
	__voltagePublisher = None
	__proximityPublisher = None

	# generic topic publisher nodes tied to switches on the ground unit
	__genericSwitchPublisher_0 = None
	__genericSwitchPublisher_1 = None
	__genericSwitchPublisher_2 = None
	__genericSwitchPublisher_3 = None

	# generic topic subscriber nodes tied to LEDs on the ground unit 
	__genericLEDSubscriber_0 = None
	__genericLEDSubscriber_1 = None
	__genericLEDSubscriber_2 = None
	__genericLEDSubscriber_3 = None
	__genericLEDSubscriber_4 = None
	__genericLEDSubscriber_5 = None
	__genericLEDSubscriber_6 = None
	__genericLEDSubscriber_7 = None

	__LEDflag_0 = 0
	__LEDflag_1 = 0
	__LEDflag_2 = 0
	__LEDflag_3 = 0
	__LEDflag_4 = 0
	__LEDflag_5 = 0
	__LEDflag_6 = 0
	__LEDflag_7 = 0

	__switchList = None

	# Name:		Default constructor
	# Description:	ROSNodeManager class default constructor which initializes the node and then creates a number
	# 				of other nodes, both publishers and subscribers, by calling the various initializer methods
	# Parameters:	None
	# Return:		None
	def __init__(self):

		rospy.init_node('groundUnitNode')
		self.initializeVoltageNode()
		self.initializeProximityNode()
		self.initializeGenericSwitchNodes()
		self.initializeGenericLEDNodes()

	# Name:		initializeVoltageNode
	# Description:	ROSNodeManager class method which initializes the voltage publisher node which 
	# 				continually publishes the voltage value  
	# Parameters:	None
	# Return:		None
	def initializeVoltageNode(self):

		self.__voltagePublisher = rospy.Publisher('voltagePublish', std_msgs.msg.String, queue_size=1)

	# Name:		initializeVoltageNode
	# Description:	ROSNodeManager class method which initializes the proximity publisher node which 
	# 				continually publishes the proximity values  
	# Parameters:	None
	# Return:		None
	def initializeProximityNode(self):

		# for publishing the proximity as a string
		#self.__proximityPublisher = rospy.Publisher('proximityPublish', std_msgs.msg.String, queue_size=1)

		# for publishing the proximity as a Marker object
		self.__proximityPublisher = rospy.Publisher('proximityPublish', Marker, queue_size=10)

	# Name:		initializeGenericSwitchNodes
	# Description:	ROSNodeManager class method which initializes four generic publisher nodes which send
	# 				signals corresponding to the state of toggle switches on the ground unit
	# Parameters:	None
	# Return:		None
	def initializeGenericSwitchNodes(self):	

		self.__genericSwitchPublisher_0 = rospy.Publisher('genericSwitchPublish_0', std_msgs.msg.Int8, queue_size=1)
		self.__genericSwitchPublisher_1 = rospy.Publisher('genericSwitchPublish_1', std_msgs.msg.Int8, queue_size=1)
		self.__genericSwitchPublisher_2 = rospy.Publisher('genericSwitchPublish_2', std_msgs.msg.Int8, queue_size=1)
		self.__genericSwitchPublisher_3 = rospy.Publisher('genericSwitchPublish_3', std_msgs.msg.Int8, queue_size=1)

	# Name:		initializeGenericLEDNodes
	# Description:	ROSNodeManager class method which initializes eight generic subscriber nodes which listen
	# 				for ROS topics which cause LEDs on the ground unit to be activated if a generic alert is sent
	# 				if the flags are high   
	# Parameters:	None
	# Return:		None
	def initializeGenericLEDNodes(self):	

		self.__genericLEDSubscriber_0 = rospy.Subscriber('genericLEDNode_0', std_msgs.msg.Int8, self.receiveLEDFlag_0)
		self.__genericLEDSubscriber_1 = rospy.Subscriber('genericLEDNode_1', std_msgs.msg.Int8, self.receiveLEDFlag_1)
		self.__genericLEDSubscriber_2 = rospy.Subscriber('genericLEDNode_2', std_msgs.msg.Int8, self.receiveLEDFlag_2)
		self.__genericLEDSubscriber_3 = rospy.Subscriber('genericLEDNode_3', std_msgs.msg.Int8, self.receiveLEDFlag_3)
		self.__genericLEDSubscriber_4 = rospy.Subscriber('genericLEDNode_4', std_msgs.msg.Int8, self.receiveLEDFlag_4)
		self.__genericLEDSubscriber_5 = rospy.Subscriber('genericLEDNode_5', std_msgs.msg.Int8, self.receiveLEDFlag_5)
		self.__genericLEDSubscriber_6 = rospy.Subscriber('genericLEDNode_6', std_msgs.msg.Int8, self.receiveLEDFlag_6)
		self.__genericLEDSubscriber_7 = rospy.Subscriber('genericLEDNode_7', std_msgs.msg.Int8, self.receiveLEDFlag_7)

	# Names:		receiveLEDFlag_0 - receiveLEDFlag_7
	# Description:	ROSNodeManager class call back methods which subscribe to topics from nodes called 
	# 				"genericLEDNode_0" to "genericLEDNode_7" and activates LEDs on the ground unit
	# 				if the flags are high   
	# Parameters:	Take in an 8 bit int which acts as a flag to signal that a generic alert has been 
	# 				received
	# Return:		None
	def receiveLEDFlag_0(self, message):

		print(message.data)
		self.__LEDflag_0 = message.data

	def receiveLEDFlag_1(self, message):

		print(message.data)
		self.__LEDflag_1 = message.data

	def receiveLEDFlag_2(self, message):
		print(message.data)

		self.__LEDflag_2 = message.data

	def receiveLEDFlag_3(self, message):
		print(message.data)
		self.__LEDflag_3 = message.data

	def receiveLEDFlag_4(self, message):
		print(message.data)
		self.__LEDflag_4 = message.data

	def receiveLEDFlag_5(self, message):
		print(message.data)
		self.__LEDflag_5 = message.data

	def receiveLEDFlag_6(self, message):
		print(message.data)
		self.__LEDflag_6 = message.data

	def receiveLEDFlag_7(self, message):
		print(message.data)
		self.__LEDflag_7 = message.data
		

	# Name:		    convertSensorDesc
	# Description:	ROSNodeManager class method which takes in a sensor identifier as a char, converts
	# 				it to an ordinal and appends it to the end of the quaternion label, which is returned
	# 				as a string so that the correct sensor on-board the drone can be referenced for the 
	# 				quaternion launch file builder
	# Parameters:	Takes in an identifier to convert to an ordinal
	# Return:		Returns a string which is a quaternion label that can be used when relating the current
	# 				proximity reading with the corresponding sensor on-board the drone
	def convertSensorDesc(self, identifier):

		return "/quat/tof_tf_" + str(ord(identifier) - ord('a') + 1)

	# Name:		    publishSensorValue
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
				
                # publishing the proximity as a Marker object
				val = ord( sensorValue[0] ) - ord('a') + 1
				marker = Marker()
				marker.header.frame_id = self.convertSensorDesc(sensorValue[0])
				
				marker.header.stamp = rospy.Time()
				marker.ns = "tof_system"
				marker.id = ord(sensorValue[0])
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
				# self.__proximityPublisher.publish(str(sensorValue))

	# Name:		    publishGenericSwitches
	# Description:	ROSNodeManager class method which iterates through the list containing the states of 
	# 				toggle switches on the ground unit and publishes each of their states 
	# Parameters:	Takes in a list of values corresponding to the state of the toggle switches on the 
	# 				ground unit
	# Return:		None
	def publishGenericSwitches(self, switchList):
		
		index = 0
		size = len(switchList)
		while index < size:
			self.publishSwitchValue(index, switchList[index])
			index += 1

	# Name:		publishSwitchValue
	# Description:	ROSNodeManager class method which publishes the state of a toggle switche
	# 				on the ground unit at a particular position in the list that holds their states
	# Parameters:	Takes in an int which is the index in the generic switch list and a value
	# 				then publishes that value to the generic alert at the index specified
	# Return:		None
	def publishSwitchValue(self, index, switchValue):
		
		if index == 0:
			self.__genericSwitchPublisher_0.publish(switchValue)		
		elif index == 1:
			self.__genericSwitchPublisher_1.publish(switchValue)
		elif index == 2:
			self.__genericSwitchPublisher_2.publish(switchValue)				
		elif index == 3:
			self.__genericSwitchPublisher_3.publish(switchValue)

	# Name:		    subscribeGenericLEDs
	# Description:	ROSNodeManager class method which creates a list of ints, iterates through the flags
	# 				tied to generic alerts which are being subscribed to, and sets the ints to the states
	# 				of the alerts flags, and returns the list, which is used to determine which LEDs on the
	# 				ground unit to activate
	# Parameters:	None
	# Return:		Returns a list of flags corresponding to the generic alerts tied to LEDs on the ground unit
	def subscribeGenericLEDs(self):
		
		hexValue = self.__HEX_ZERO
		index = self.__ZERO
		size = self.__HEX_SIZE
		
		while index < size:	
			if self.subscribeLEDValue(index) != self.__ZERO:
				hexValue |= self.__HEX_ONE
			if index < size - 1:
				hexValue = hexValue << 1	
				
			index += 1

		return hexValue	

	# Name:		    subscribeLEDValue
	# Description:	ROSNodeManager class method which initializes eight generic subscriber nodes which listen
	# 				for ROS topics which cause LEDs on the ground unit to be activated if a generic alert is sent
	# 				if the flags are high   
	# Parameters:	Takes in an int which is the index in a list and checks the generic LED related to the
	# 				the flag of that index value, and then returns the value of the flag
	# Return:		None
	def subscribeLEDValue(self, index):

		if index == 0:
			return self.__LEDflag_0 
		elif index == 1:
			return self.__LEDflag_1
		elif index == 2:
			return self.__LEDflag_2 
		elif index == 3:
			return self.__LEDflag_3 
		elif index == 4:
			return self.__LEDflag_4 
		elif index == 5:
			return self.__LEDflag_5 
		elif index == 6:
			return self.__LEDflag_6 
		elif index == 7:
			return self.__LEDflag_7
			
