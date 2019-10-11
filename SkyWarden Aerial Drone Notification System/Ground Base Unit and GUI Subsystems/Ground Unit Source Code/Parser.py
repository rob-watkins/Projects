
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	Parser.py 
# Description:	Parser class implementation	
#				Parser class parses, reassembles, and formats input from the subsystem on-board
#				the drone coming in over the serial port from the receiver for use in the ground 
#				base unit   

from SerialPort import SerialPort

class Parser:

	__serialPort = None
	__parsedInput = None
	__serialReadSize = None

	# Name:			Parameterized constructor
	# Description:	Parser class parameterized constructor which takes in baud rate and timeout
	# 				values which are used to create a SerialPort object
	# Parameters:	Takes in two ints which are the baud rate and timeout value needed in the 
	# 				constructor of the SerialPort object
	# Return:		None
	def __init__(self, baudRate, timeOut, readinSize):

		self.__serialReadSize = readinSize

		# SerialPort object created with baud rate and timeout values passed into the constructor 
		self.__serialPort = SerialPort(baudRate, timeOut, readinSize) 


	# Name:			getSerialInput
	# Description:	Parser class method which continually reads in from the serial port through 
	# 				the SerialPort object and formats and concatenates the portions of the input, 
	# 				removing artifacts added by PySerial and checking for a decimal point
	# Parameters:	None
	# Return:		Returns a list containing the value from the receiver as a string and char
	# 				indicating what type of value it is
	def getSerialInput(self):
		
		dotBool = False

		# for reading in an entire line at a time from the serial port
		if self.__serialReadSize == 1:
			
			# reads in an entire line from the serial port
			inputValue = str(self.__serialPort.serialRead())
			
			# leading and trailing characters are parsed off the line and the outputList is built 
			# from the parsed line
			outputList = [inputValue[2]]
			outputString = inputValue[3:-4]
			outputString = outputString.replace('\\', "")
			outputList.append(outputString)

		# for reading in data from the serial port one byte at a time
		if read.__serialReadSize == 0:

			# grabbing the first portion of the input, which enters in the format: b'1'
			# where 1 is the value coming in, the final values being: b'\', b'r', b'\', b'n'
			# which, along with the 'b', and ''' chars, are removed
			
			inputValue = str(self.__serialPort.serialRead())
			
			inputValue.split("'")
			value = ""

			# while the first '\' char of the carriage return, \r\n, has not been encountered, the 
			# the first char in the string is retained and outputList is built from each one
			
			while inputValue[2] != "\\":
				# skips errant, additional '.' chars if they are present in the input
				if inputValue[2] == '.':
					if not dotBool:
						dotBool = True
						value += inputValue[2]
				else:
					value += inputValue[2] 	

				inputValue = str(self.__serialPort.serialRead())
				inputValue.split("'")

			# a list is built from the input, making the first char in the input the first
			# element in the list which is a control char which marks whether the input is a
			# voltage, with a 'v', or as a proximity reading, with any char from 'a' to 'n' 
			# and the second element the input value itself		
			if len(value) > 1:
				outputList = [str(value[0]), str(value[1:])]
			else:
				outputList = value 

			inputValue = str(self.__serialPort.serialRead())
		
		# method call to trimValue to trim excess trailing digits from the input which will
		# then be returned
		outputList = self.trimValue(outputList)
		
		# the serial port buffer is flushed
		self.__serialPort.flushSerialPort()
		
		return outputList


	# Name:			trimValue
	# Description:	Parser class method which takes in a list containing as the second element
	# 				a string which is to have trailing digits trimmed off depending whether the 
	# 				string corresponds to a voltage or a proximity reading
	# Parameters:	Takes in a list containing a control char marking the type of value which is
	# 				being recieved from the serial port, which is the second element in the list
	# Return:		Returns the list that was taken in but with the string containing the value
	# 				trimmed down to the proper size for use in other classes in the ground unit
	def trimValue(self, outputList):
		
		index = 1
		voltLimit = 6
		proxLimit = 5

		if outputList != None and len(outputList) > 1 and len(outputList[1]) > 0:
			listSize = len(outputList[1])

			# if the first element in the list is not a valid control then return None
			if outputList[0] < 'a' or outputList[0] > 'z':
				return None

			# the input value is iterated through and there are non-numeric chars in the
			# string, return None
			while index < listSize:
				if outputList[1][index] >= 'a' and outputList[1][index] <= 'z':
					return None
				index += 1

			# if the input value is a voltage, enforce a string size of 5
			if outputList[0] == 'v' and listSize > 6:
				outputList[1] = outputList[1][:-(listSize - voltLimit)]
			# else if the input value is proximity, enforce a string size of 4
			elif len(outputList) > 0 and outputList[0] >= 'a' and outputList[0] <= 'l' and listSize > 5:
				outputList[1] = outputList[1][:-(listSize- proxLimit)]

		return outputList


	# Name:			parsePipeInput
	# Description:	Parser class method which takes in a value and immediately returns it; provides for
	# 				further parsing if necessary in the future by way of keeping other parsing methods
	# 				intact
	# Parameters:	Takes in a value which is forwarded
	# Return:		Returns the value passed into the method
	def parsePipeInput(self, inputValue):

		return inputValue	

