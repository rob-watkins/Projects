# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	quaternion_loader_test.py 
# Description:	This is a simple test to ensure the QuaternionManager class can properly load
#				a comma delimitted list of values and create an xml launch file from those values

from quaternion_loader import QuaternionManager

# class object declaration
test = QuaternionManager()

# class method loadValues is the only call necessary
test.loadValues("sensorOffsets.test")
