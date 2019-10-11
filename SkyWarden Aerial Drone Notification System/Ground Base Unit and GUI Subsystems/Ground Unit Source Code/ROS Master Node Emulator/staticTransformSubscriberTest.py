# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	staticTransformySubscriberTest.py 
# Description:	Test module for ROS master node emulation which tests the now depreciated static transform
#               message publisher feature which has been replaced by the QuaternionManager class which 
#               builds an XML file which launches the transform upon start up of the system rather than
#               continually

import tf
import tf2_ros
import tf2_py
import rospy
import geometry_msgs.msg 

class StaticTransformSubscriber(object):

    def __init__(self):
        rospy.init_node('staticTransformSubscriber')

        self.transformMessage = geometry_msgs.msg.TransformStamped()
        rospy.Subscriber('staticTransformPublish', geometry_msgs.msg.TransformStamped, self.receiveTransform)

    def receiveTransform(self, message):
        self.transformMessage = message

    def run(self):
        rate = rospy.Rate(10)

        while self.transformMessage is None and not rospy.is_shutdown():
            print "waiting" 
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            rospy.loginfo("Time stamp: " + str(self.transformMessage.header.stamp))
            rospy.loginfo("Sequence number: " + str(self.transformMessage.header.seq))
            rospy.loginfo("Header frame ID: " + self.transformMessage.header.frame_id)
            rospy.loginfo("Child frame ID: " + self.transformMessage.child_frame_id)
            rospy.loginfo("Translation x: " + str(self.transformMessage.transform.translation.x))
            rospy.loginfo("Translation y: " + str(self.transformMessage.transform.translation.y))
            rospy.loginfo("Translation z: " + str(self.transformMessage.transform.translation.z))
            rospy.loginfo("Rotation x: " + str(self.transformMessage.transform.rotation.x))
            rospy.loginfo("Rotation y: " + str(self.transformMessage.transform.rotation.y))
            rospy.loginfo("Rotation z: " + str(self.transformMessage.transform.rotation.z))
            rospy.loginfo("Rotation w: " + str(self.transformMessage.transform.rotation.w))

            rate.sleep()

node = StaticTransformSubscriber()
node.run()
