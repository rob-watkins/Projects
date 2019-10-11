
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	GroundUnitDriver.py 
# Description:	Main driver for ground unit (Headless ANDS)
#				Main driver which simply reads in the data streamed in from the drone, parses it, and 
# 				publishes it as ROS topics

from time import sleep
from threading import Thread
from Parser import Parser
from ROSNodeManager import ROSNodeManager
from quaternion_loader import QuaternionManager
import subprocess

# main loop, thread, constants and variables

INITIALIZATION_ONE = 1
BAUD_RATE = 1000000
TIME_OUT = 0.5
PARSE_CONTROL = 1
BIT = 0
BYTE = 1

# class object creation for the Parser, ROSNodeManager, SevenSegment, and
# ComponentManager objects used in the main loop, threads, and processes
parser = Parser(BAUD_RATE, TIME_OUT, BYTE)
nodeManager = ROSNodeManager()


# Name:			quaternionLauncher
# Description:	Free function which runs concurrently with the main loop in a separate thread
# 				which creates a roslaunch XML file from the sensor offsets configuration file
# 				and executes a system call to launch the XML to publish the quaternion values
# Parameters:	None
# Return:		None
def quaternionLauncher():
	
	quaternionManager = QuaternionManager()
	quaternionManager.loadValues("sensorOffsets.config")	
	quaternionManager.launchWriter()
	
	subprocess.call(['roslaunch', 'quaternion.launch'])

quaternionThread = Thread(target=quaternionLauncher)
quaternionThread.start()

sleep(INITIALIZATION_ONE)

# the main loop which continually receives data through the serial port and publishes it
# through ROS
while True:

	# the Parser class object method to get values from the serial port is invoked to bring
	# data in from the drone
	value = parser.getSerialInput() 

	# voltage and proximity values are publish to ROS, and generic switch and LED alerts are
	# published and subscribed to  
	if value != None and len(value) > 1 and (value[0] == 'v' or ((value[0] >= 'a' and value[0] <= 'n') and value[1].isnumeric() and int(value[1]) < 2800)):
		nodeManager.publishSensorValue(value)
	

