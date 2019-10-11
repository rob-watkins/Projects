# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	genericLEDPublisher.py 
# Description:	Test module for ROS master node emulation which tests the generic LEDs on the ground base
#               unit by continually publishing eight ROS topics which correspond to the eight generic alerts
#               which can be added to the system by the user; the value 0x8E is published to the ground
#               unit which allows the generic LED feature to be tested

import rospy
import std_msgs.msg

rospy.init_node('genericLEDNodes')

genericLEDNode_0 = rospy.Publisher('genericLEDNode_0', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_1 = rospy.Publisher('genericLEDNode_1', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_2 = rospy.Publisher('genericLEDNode_2', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_3 = rospy.Publisher('genericLEDNode_3', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_4 = rospy.Publisher('genericLEDNode_4', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_5 = rospy.Publisher('genericLEDNode_5', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_6 = rospy.Publisher('genericLEDNode_6', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_7 = rospy.Publisher('genericLEDNode_7', std_msgs.msg.Int8, queue_size=1)

while(True):
	print("Publishing generic alerts 0 though 7")
	genericLEDNode_0.publish(1)
	genericLEDNode_1.publish(0)
	genericLEDNode_2.publish(0)
	genericLEDNode_3.publish(0)
	genericLEDNode_4.publish(1)
	genericLEDNode_5.publish(1)
	genericLEDNode_6.publish(1)
	genericLEDNode_7.publish(0)
