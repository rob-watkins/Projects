# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	mainROSNodeTest.py 
# Description:	Test module for ROS master node emulation which builds a scaled down version of the system
#               so that the ROS master emulator can be first be tested in order to ensure the tests are
#               correct; also acts as prototyping and scaffolding for the ROS components in the completed 
#               system itself               

import time
import tf
import tf2_ros
import tf2_py
import roslaunch
import rospy
import std_msgs.msg
import geometry_msgs.msg
from threading import Thread

voltage = 14.0
dataList = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
distances = std_msgs.msg.Int32MultiArray(data=dataList)

class GenericLEDSubscriber(object):

    def __init__(self):

        self.flag_0 = 0
        self.flag_1 = 0
        self.flag_2 = 0
        self.flag_3 = 0
        self.flag_4 = 0
        self.flag_5 = 0
        self.flag_6 = 0
        self.flag_7 = 0

        rospy.Subscriber('genericLEDNode_0', std_msgs.msg.Int8, self.receiveFlag_0)
        rospy.Subscriber('genericLEDNode_1', std_msgs.msg.Int8, self.receiveFlag_1)
        rospy.Subscriber('genericLEDNode_2', std_msgs.msg.Int8, self.receiveFlag_2)
        rospy.Subscriber('genericLEDNode_3', std_msgs.msg.Int8, self.receiveFlag_3)
        rospy.Subscriber('genericLEDNode_4', std_msgs.msg.Int8, self.receiveFlag_4)
        rospy.Subscriber('genericLEDNode_5', std_msgs.msg.Int8, self.receiveFlag_5)
        rospy.Subscriber('genericLEDNode_6', std_msgs.msg.Int8, self.receiveFlag_6)
        rospy.Subscriber('genericLEDNode_7', std_msgs.msg.Int8, self.receiveFlag_7)

        self.update = Thread(target=self.run)
        self.update.start()   
        
    def receiveFlag_0(self, message):
        self.flag_0 = message

    def receiveFlag_1(self, message):
    	self.flag_1 = message

    def receiveFlag_2(self, message):
    	self.flag_2 = message

    def receiveFlag_3(self, message):
    	self.flag_3 = message

    def receiveFlag_4(self, message):
    	self.flag_4 = message

    def receiveFlag_5(self, message):
    	self.flag_5 = message

    def receiveFlag_6(self, message):
    	self.flag_6 = message

    def receiveFlag_7(self, message):
    	self.flag_7 = message

    def run(self):
        rate = rospy.Rate(1000)
        #remove-able
        while self.flag_0 is None and self.flag_1 is None and self.flag_2 is None and self.flag_3 is None \
        and self.flag_4 is None and self.flag_5 is None and self.flag_6 is None and self.flag_7 is None \
        and not rospy.is_shutdown():
            print "waiting" 
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            rospy.loginfo("Generic LED flag 0: " + str(self.flag_0))
            rospy.loginfo("Generic LED flag 1: " + str(self.flag_1))
            rospy.loginfo("Generic LED flag 2: " + str(self.flag_2))
            rospy.loginfo("Generic LED flag 3: " + str(self.flag_3))
            rospy.loginfo("Generic LED flag 4: " + str(self.flag_4))
            rospy.loginfo("Generic LED flag 5: " + str(self.flag_5))
            rospy.loginfo("Generic LED flag 6: " + str(self.flag_6))
            rospy.loginfo("Generic LED flag 7: " + str(self.flag_7))

            rate.sleep()

class StaticTransformPublisher(object):

	def __init__(self):

		# static transform publisher test setup
		# staticTransfromPublisher = tf2_ros.StaticTransformBroadcaster()
		self.staticTransfromPublisher = rospy.Publisher('staticTransformPublish', geometry_msgs.msg.TransformStamped, queue_size=1)

		self.transformMessage = geometry_msgs.msg.TransformStamped()
		self.quaternion = tf.transformations.quaternion_from_euler(4.0, 5.0, 6.0)
		self.transformMessage.header.stamp = rospy.Time.now()
		self.transformMessage.header.seq = 0
		self.transformMessage.header.frame_id = "header frame"
		self.transformMessage.child_frame_id = "child frame"
		self.transformMessage.transform.translation.x = float(1.0)
		self.transformMessage.transform.translation.y = float(2.0)
		self.transformMessage.transform.translation.z = float(3.0)
		self.transformMessage.transform.rotation.x = self.quaternion[0]
		self.transformMessage.transform.rotation.y = self.quaternion[1]
		self.transformMessage.transform.rotation.z = self.quaternion[2]
		self.transformMessage.transform.rotation.w = self.quaternion[3]

		# launch file test
		# filePath = "some path"
		# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		# roslaunch.configure_logging(uuid)
		# launch = roslaunch.parent.ROSLaunchParent(uuid, [filePath])
		# launch.start()

class VoltagePublisher(object):

	def __init__(self):

		# voltage publisher test setup
		self.voltagePublisher = rospy.Publisher('voltagePublish', std_msgs.msg.Float32, queue_size=1)

class ProximityPublisher(object):

	def __init__(self):

		# proximity publisher test setup
		# proximityPublisher = rospy.Publisher('proximityPublish', std_msgs.msg.Int16, queue_size=1)
		self.proximityPublisher = rospy.Publisher('proximityPublish', std_msgs.msg.Int32MultiArray, queue_size=1)

class GenericSwitchPublisher(object):

	def __init__(self):

		# proximity publisher test setup
		# proximityPublisher = rospy.Publisher('proximityPublish', std_msgs.msg.Int16, queue_size=1)
		self.genericSwitchPublisher_0 = rospy.Publisher('genericSwitchPublish_0', std_msgs.msg.Int8, queue_size=1)
		self.genericSwitchPublisher_1 = rospy.Publisher('genericSwitchPublish_1', std_msgs.msg.Int8, queue_size=1)
		self.genericSwitchPublisher_2 = rospy.Publisher('genericSwitchPublish_2', std_msgs.msg.Int8, queue_size=1)
		self.genericSwitchPublisher_3 = rospy.Publisher('genericSwitchPublish_3', std_msgs.msg.Int8, queue_size=1)

	def run(self):
		
		self.genericSwitchPublisher_0.publish(1)
		self.genericSwitchPublisher_1.publish(0)
		self.genericSwitchPublisher_2.publish(1)
		self.genericSwitchPublisher_3.publish(0)	

rospy.init_node('mainNode')
genericLEDNode = GenericLEDSubscriber()
transformNode = StaticTransformPublisher()
voltageNode = VoltagePublisher()
proximityNode = ProximityPublisher()
genericSwitchNode = GenericSwitchPublisher()

while(True):
	#print "hi"
	#staticTransfromPublisher.sendTransform(transformMessage)
	transformNode.staticTransfromPublisher.publish(transformMessage)
	voltageNode.voltagePublisher.publish(voltage)
	# for index in distances:
	# 	print str(distances[index - 1])
	# 	proximityPublisher.publish(distances[index - 1])
	proximityNode.proximityPublisher.publish(distances)
	genericSwitchNode.run()
