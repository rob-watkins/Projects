# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	proximitySubscriberTest.py 
# Description:	Test module for ROS master node emulation which tests the proximity publisher on the ground base
#               base unit by continually subscribing to the 14 ROS topics which correspond to the proximity
#               values streaming in from the drone

import rospy
import std_msgs.msg

class ProximitySubscriber(object):

    def __init__(self):
        rospy.init_node('proximitySubscriber')

        self.distances = None
        rospy.Subscriber('proximityPublish', std_msgs.msg.String, self.receiveVoltage)

    def receiveVoltage(self, message):
        self.distances = message

    def run(self):
        rate = rospy.Rate(1000)

        while self.distances is None and not rospy.is_shutdown():
            print ("waiting")
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            rospy.loginfo("Distance: " + str(self.distances))

            rate.sleep()

node = ProximitySubscriber()
node.run()
