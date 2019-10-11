# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	parameterizedGenericLEDPublisher.py 
# Description:	Test module for ROS master node emulation which tests the generic LEDs on the ground base
#               unit by continually publishing eight ROS topics which correspond to the eight generic alerts
#               which can be added to the system by the user; the value that is published to the ground
#               unit is taken from the command line which allows different values to be entered and for
#               the generic LED feature to be tested with a range of different values

import sys
import rospy
import std_msgs.msg

LED_0 = 0
LED_1 = 0
LED_2 = 0
LED_3 = 0
LED_4 = 0
LED_5 = 0
LED_6 = 0
LED_7 = 0

argumentLength = len(sys.argv)
print(argumentLength)

rospy.init_node('genericLEDNodes')

genericLEDNode_0 = rospy.Publisher('genericLEDNode_0', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_1 = rospy.Publisher('genericLEDNode_1', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_2 = rospy.Publisher('genericLEDNode_2', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_3 = rospy.Publisher('genericLEDNode_3', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_4 = rospy.Publisher('genericLEDNode_4', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_5 = rospy.Publisher('genericLEDNode_5', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_6 = rospy.Publisher('genericLEDNode_6', std_msgs.msg.Int8, queue_size=1)
genericLEDNode_7 = rospy.Publisher('genericLEDNode_7', std_msgs.msg.Int8, queue_size=1)


if argumentLength > 1:
	print(sys.argv[1])
	if sys.argv[1] == "0":
		LED_0 = 0
	else:
		LED_0 = 1	
		
if argumentLength > 2:
	print(sys.argv[2])
	if sys.argv[2] == "0":
		LED_1 = 0
	else:
		LED_1 = 1
		
if argumentLength > 3:
	print(sys.argv[3])
	if sys.argv[3] == "0":
		LED_2 = 0
	else:
		LED_2 = 1
		
if argumentLength > 4:
	print(sys.argv[4])
	if sys.argv[4] == "0":
		LED_3 = 0
	else:
		LED_3 = 1
		
if argumentLength > 5:
	print(sys.argv[5])
	if sys.argv[5] == "0":
		LED_4 = 0
	else:
		LED_4 = 1
		
if argumentLength > 6:
	print(sys.argv[6])
	if sys.argv[6] == "0":
		LED_5 = 0
	else:
		LED_5 = 1
											
if argumentLength > 7:
	print(sys.argv[7])
	if sys.argv[7] == "0":
		LED_6 = 0
	else:
		LED_6 = 1
		
if argumentLength > 8:
	print(sys.argv[8])
	if sys.argv[8] == "0":
		LED_7 = 0
	else:
		LED_7 = 1

print(LED_0)
print(LED_1)
print(LED_2)
print(LED_3)
print(LED_4)
print(LED_5)
print(LED_6)
print(LED_7)

while(True):
	genericLEDNode_0.publish(int(LED_0))
	genericLEDNode_1.publish(int(LED_1))
	genericLEDNode_2.publish(int(LED_2))
	genericLEDNode_3.publish(int(LED_3))
	genericLEDNode_4.publish(int(LED_4))
	genericLEDNode_5.publish(int(LED_5))
	genericLEDNode_6.publish(int(LED_6))
	genericLEDNode_7.publish(int(LED_7))

