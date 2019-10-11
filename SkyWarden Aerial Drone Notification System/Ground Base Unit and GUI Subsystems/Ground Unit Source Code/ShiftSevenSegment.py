
# Project: 		CS426 Spring 2018, Team #23: SkyWarden ADNS Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	ShiftSevenSegment.py 
# Description:	ShiftSevenSegment class implementation	
#				SevenSegment class establishes two, four digit seven segment displays,
#				and continually writes four digit values on each display, values 
#				corresponding to the voltage from the drone and the proximity threshold set in
#				the GUI, created for the 5641AS Common Anode Seven Segment Display

# Hardware:
# Seven Segment Display:			5641AS Common Anode Seven Segment Display
# 3x8 Multiplexer:					SN74LS138 3x8 1-of-8 Decoder/Demultiplexer
# 8 Bit Shift Registor:				SN74LS164N 8-Bit In/Parallel Out Shift Register
# 8 Bit D-Latch:					SN74LS373N Octal D-Type Latches

from gpiozero import LED
from time import sleep

class ShiftSevenSegment:

	# constant for pins on the Raspberry Pi
	__CLOCK_PIN = 26
	__ENABLE_PIN = 16
	__VOLTAGE_PIN = 19
	__THRESHOLD_PIN = 13
	__DIGIT_ZERO_PIN = 21
	__DIGIT_ONE_PIN = 20
	
	# constants for the segment values in hexadecimal
	__HEX_ZERO = 0x00
	__HEX_ONE = 0x01
	__HEX_SIZE = 8
	__ZERO = 0x7E
	__ONE = 0x30
	__TWO = 0x6D
	__THREE = 0x79
	__FOUR = 0x33
	__FIVE = 0x5B
	__SIX = 0x5F
	__SEVEN = 0x70
	__EIGHT = 0x7F
	__NINE = 0x73
	__DECIMAL_POINT = 0x80
	
	__DISPLAY_DELAY = 0.005
	__PERIOD_CHAR = '.'

	# creation of LED objects
	clock = LED(__CLOCK_PIN)
	enable = LED(__ENABLE_PIN)
	voltageDisplay = LED(__VOLTAGE_PIN)
	thresholdDisplay = LED(__THRESHOLD_PIN)
	digitSelect_0 = LED(__DIGIT_ZERO_PIN)
	digitSelect_1 = LED(__DIGIT_ONE_PIN)

# Seven Segment Map:
#
#		Hexadecmial Bit:		Segment: 		
#		0x80					DP (decimal point)		
#		0x40					a
#		0x20					b
#		0x10					c
#		0x01					d
#		0x02					e
#		0x04					f
#		0x01					g


	# Name:			Default constructor
	# Description:	ShiftSevenSegment class default constructor which initializes the LED
	# 				objects for the clock, enable, data, and digit select inputs to off 
	# Parameters:	None
	# Return:		None
	def __init__(self):
		
		self.clock.off()
		self.enable.off()

		self.thresholdDisplay.off()
		self.voltageDisplay.off()

		self.digitSelect_0.off()
		self.digitSelect_1.off()

	# Name:			selectDigit
	# Description:	ShiftSevenSegment class method which takes in an int which is the the digit
	# 				on the display to write, writing that digit low, thereby turning it on, for 
	# 				both the voltage and the proximity threshold at the same time
	# Parameters:	Takes in an int which corresponds to the digit on the seven segment
	# 				being written to, as each digit must be written individually
	# Return:		None
	def selectDigit(self, digit):

		if digit == 0:
			self.digitSelect_0.off()
			self.digitSelect_1.off()
		if digit == 1:
			self.digitSelect_0.on()
			self.digitSelect_1.off()
		if digit == 2:
			self.digitSelect_0.off()
			self.digitSelect_1.on()
		if digit == 3:	
			self.digitSelect_0.on()
			self.digitSelect_1.on()

	# Name:			hexadecimalConversion
	# Description:	ShiftSevenSegment class method which takes a number, the numbers to write to 
	# 				a given digit on the seven segment display and then builds the hexadecimal 
	# 				value corresponding to the segments to drive high, also taking in a bool
	# 				corresponding to whether or not the decimal point should be written and if
	# 				so the hexadecimal value is altered to reflect that 
	# 				voltage value 
	# Parameters:	Takes in a char corresponding to the number to write to the current digit
	# 				on display, and a bool which signals whether or not the decimal point will 
	# 				be written
	# Return:		Returns the hexadecimal value which was built from the char and the bool
	# 				input into the method, representing the number to display
	def hexadecimalConversion(self, number, decimalPoint):
	
		hexValue = self.__HEX_ZERO

		if number == '0':
			hexValue = self.__ZERO

		if number == '1':
			hexValue = self.__ONE

		if number == '2':
			hexValue = self.__TWO

		if number == '3':
			hexValue = self.__THREE

		if number == '4':
			hexValue = self.__FOUR

		if number == '5':
			hexValue = self.__FIVE

		if number == '6':
			hexValue = self.__SIX

		if number == '7':
			hexValue = self.__SEVEN

		if number == '8':
			hexValue = self.__EIGHT

		if number == '9':
			hexValue = self.__NINE

		if decimalPoint:
			hexValue |= self.__DECIMAL_POINT

		return hexValue
	
	# Name:			shiftIn
	# Description:	ShiftSevenSegment class method which takes in two hexadecimal numbers for voltage
	# 				and proximity threshold and shifts them into the two shift registers for both
	# 				displays simultaneously
	# Parameters:	Takes in two hexadecimal numbers which are digits on the voltage and proximity
	# 				threshold displays which will be shifted into the shift register 
	# Return:		None
	def shiftIn(self, voltageHex, proximityHex):

		self.clock.off()

		for index in range(self.__HEX_SIZE):
			if (voltageHex & self.__HEX_ONE):			
				self.voltageDisplay.on()
			else:
				self.voltageDisplay.off()
			voltageHex = voltageHex >> 1
		
			if (proximityHex & self.__HEX_ONE):			
				self.thresholdDisplay.on()
			else:
				self.thresholdDisplay.off()
			proximityHex = proximityHex >> 1

			self.clock.on()
			self.clock.off()

	# Name:			displayNumber
	# Description:	ShiftSevenSegment class method which takes in two strings to write the seven
	# 				segment displays, and moves through each of them and sends each char in the
	# 				strings into another class method which builds hexadecimal values from them
	# 				and then shifts those values into the shift registers for each display, and
	# 				then latches that into two D-latches which feed into the displays 
	# Parameters:	Takes in two strings corresponding to the entire voltage and proximity
	# 				threshold values to be written to the seven segment displays	
	# Return:		None
	def displayNumber(self, voltage, proximity):	

		voltageSize = len(voltage)
		proximitySize = len(proximity)

		if(voltageSize > 0 and proximitySize > 0):
			decimal = False

			# both strings are reversed so that the least significant digits are 
			# at the front of the string  
			voltageString = "".join(reversed(voltage))
			proximityString = "".join(reversed(proximity))
			digitSelect = 0
			counter = 0

			# iterates through both strings and writes each one, digit by digit, to the
			# corresponding seven segment
			while counter < voltageSize and digitSelect < proximitySize:
			
				# if a decimal point is encountered, a bool is set and the position in
				# the string is advanced past it
				if voltageString[counter] == self.__PERIOD_CHAR:
					decimal = True 
					counter += 1

				# the appropriate digit on the seven segment is selected
				self.selectDigit(digitSelect)
				
				# the current digit in both strings is written to the seven segment displays
				voltageHex = self.hexadecimalConversion(voltageString[counter], decimal)
				proximityHex = self.hexadecimalConversion(proximityString[digitSelect], False)

				self.enable.off()
				self.shiftIn(voltageHex, proximityHex)
				self.enable.on()
				sleep(self.__DISPLAY_DELAY)

				decimal = False
				counter += 1
				digitSelect += 1

