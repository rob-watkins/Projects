
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	SerialPort.py 
# Description:	SerialPort class implementation	
#				SerialPort class looks for open serial ports, sets baud rate and timeout values
#				passed into its constructor, and then creates a Serial object from the PySerial
#				library which is used to stream in from the serial port which is how the data 
#				from the drone is coming into the ground unit

import serial
from serial.tools import list_ports
from subprocess import call 

class SerialPort:

	__portName = None
	__serialValue = None
	__serialReadSize = None

	# Name:			Parameterized constructor
	# Description:	SerialPort class parameterized constructor which takes in baud rate and timeout
	# 				values which are used to create set up a Serial object
	# Parameters:	Takes in two ints which are the baud rate and timeout value needed in the 
	# 				constructor of the Serial object
	# Return:		None
	def __init__(self, baudRate, timeOut, readinSize):
		
		self.__serialReadSize = readinSize

		self.__portName = self.findSerialPorts()
		call(["sudo", "chmod", "777", self.__portName])
		self.__serialValue = serial.Serial(self.__portName, baudRate, timeout=timeOut)

	# Name:			findSerialPorts	
	# Description:	SerialPort class method which iterates through all the serial ports looking for
	# 				the ports in use preceded by either "ACM" or "USB" and returns that port as a string
	# 				as the port number will often change and be assigned new values 
	# Parameters:	None
	# Return:		Returns the name of the serial port in use as a string
	def findSerialPorts(self):	

		ports = list(serial.tools.list_ports.comports())

		for portNumber, description, address in ports:
			if 'ACM' in description:
				return portNumber
			if 'USB' in description:
				return portNumber	

	# Name:			serialRead
	# Description:	SerialPort class method which forwards the value intercepted from the Serial
	# 				object on to the caller
	# Parameters:	None
	# Return:		Returns the value taken in from the serial port via the Serial object
	def serialRead(self):

		# for reading in from the serial port one byte at a time
		if self.__serialReadSize == 0:
			return self.__serialValue.read()

		# for reading in an entire line at a time from the serial port
		if self.__serialReadSize == 1:
			return self.__serialValue.readline()

	# Name:			flushSerialPort
	# Description:	SerialPort class method which invokes the Serial class method to flush the 
	# 				serial port buffer
	# Parameters:	None
	# Return:		None	
	def flushSerialPort(self):
		
		self.__serialValue.flushInput()
