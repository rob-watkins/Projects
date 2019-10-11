# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	voltageSubscriberTest.py 
# Description:	Test module for ROS master node emulation which tests the voltage publisher on the ground base
#               base unit by continually subscribing to the ROS topic which corresponds to the value
#               values streaming in from the drone

import rospy
import std_msgs.msg

class VoltageSubscriber(object):

    def __init__(self):
        rospy.init_node('voltageSubscriber')

        self.voltage = None
        rospy.Subscriber('voltagePublish', std_msgs.msg.String, self.receiveVoltage)

    def receiveVoltage(self, message):
        self.voltage = message

    def run(self):
        rate = rospy.Rate(1000)

        while self.voltage is None and not rospy.is_shutdown():
            print ("waiting")
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            rospy.loginfo("Voltage: " + str(self.voltage))

            rate.sleep()

node = VoltageSubscriber()
node.run()
