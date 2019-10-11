# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	genericSwitchSubscriber.py 
# Description:	Test module for ROS master node emulation which tests the generic switches on the ground base
#               unit by continually subscribing to four ROS topics which correspond to the four generic flag
#               switches on the ground unit

import rospy
import std_msgs.msg

class GenericSwitchSubscriber(object):

    def __init__(self):
        rospy.init_node('genericSwitchSubscriber')

        self.flag_0 = None
        self.flag_1 = None
        self.flag_2 = None
        self.flag_3 = None

        rospy.Subscriber('genericSwitchPublish_0', std_msgs.msg.Int8, self.receiveFlag_0)
        rospy.Subscriber('genericSwitchPublish_1', std_msgs.msg.Int8, self.receiveFlag_1)
        rospy.Subscriber('genericSwitchPublish_2', std_msgs.msg.Int8, self.receiveFlag_2)
        rospy.Subscriber('genericSwitchPublish_3', std_msgs.msg.Int8, self.receiveFlag_3)

    def receiveFlag_0(self, message):
        self.flag_0 = message

    def receiveFlag_1(self, message):
        self.flag_1 = message

    def receiveFlag_2(self, message):
        self.flag_2 = message

    def receiveFlag_3(self, message):
        self.flag_3 = message

    def run(self):
        rate = rospy.Rate(1000)

        while self.flag_0 is None and self.flag_1 is None and self.flag_2 is None and self.flag_3 is None and not rospy.is_shutdown():
            print ("waiting")
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            rospy.loginfo("Generic switch flag 0: " + str(self.flag_0))
            rospy.loginfo("Generic switch flag 1: " + str(self.flag_1))
            rospy.loginfo("Generic switch flag 2: " + str(self.flag_2))
            rospy.loginfo("Generic switch flag 3: " + str(self.flag_3))

            rate.sleep()

node = GenericSwitchSubscriber()
node.run()
