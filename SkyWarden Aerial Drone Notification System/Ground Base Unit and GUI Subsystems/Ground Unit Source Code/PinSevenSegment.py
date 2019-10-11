
# Project: 		CS426 Spring 2018, Team #23: SkyWarden ADNS Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	PinSevenSegment.py 
# Description:	PinSevenSegment class implementation	
#				SevenSegment class establishes one four digit seven segment displays
#				and continually writes four digit values to it, values corresponding to 
#				the voltage from the drone created for the 5641AS Common Anode Seven Segment 
#				Display, acts as backup for the ShiftSevenSegment class in the event the
#				the module containing two seven segment displays controlled by a shift register
#				and D-latch fail

# Hardware:
# Seven Segment Display:			5641AS Common Anode Seven Segment Display

from gpiozero import LED
from time import sleep

class PinSevenSegment:

	# constant for pins on the Raspberry Pi
	__D0_PIN = 27
	__D1_PIN = 22
	__D2_PIN = 23
	__D3_PIN = 24
	__DP_PIN = 21
	__A_PIN = 20
	__B_PIN = 16
	__C_PIN = 26
	__D_PIN = 19
	__E_PIN = 13
	__F_PIN = 6
	__G_PIN = 5
	
	__PERIOD_CHAR = '.'
	__DISPLAY_DELAY = 0.005
	__MAX_DIGITS = 4
	
	# creation of pin objects
	__D0 = LED(__D0_PIN)
	__D1 = LED(__D1_PIN)
	__D2 = LED(__D2_PIN)
	__D3 = LED(__D3_PIN)

	__A = LED(__A_PIN)
	__B = LED(__B_PIN)
	__C = LED(__C_PIN)
	__D = LED(__D_PIN)
	__E = LED(__E_PIN)
	__F = LED(__F_PIN)
	__G = LED(__G_PIN)
	__DP = LED(__DP_PIN)
	
	# Name:			Default constructor
	# Description:	PinSevenSegment class default constructor which initializes the display
	#				by zeroing out the pins and digits
	# Parameters:	None
	# Return:		None
	def __init__(self):
		
		self.allPinsOff()
		self.zero()

	# Name:			allPinsOff
	# Description:	PinSevenSegment class method which writes all digit pins high which turns them
	# 				off
	# Parameters:	None
	# Return:		None
	def allPinsOff(self):
		
		self.__D0.on()
		self.__D1.on()
		self.__D2.on()
		self.__D3.on()

	# Name:			zero
	# Description:	PinSevenSegment class method which writes all segment pins low which turns
	# 				them off, zeroing out the digit
	# Parameters:	None
	# Return:		None
	def zero(self):

		self.__A.off()
		self.__B.off()
		self.__C.off()
		self.__D.off()
		self.__E.off()
		self.__F.off()
		self.__G.off()
		self.__DP.off()

	# Name:			selectDigit
	# Description:	PinSevenSegment class method which takes in the digit to be activated, turns
	# 				all digits off, and then activates the the one passed in as a parameter 
	# Parameters:	Takes in an int corresponding to the digit on the display which will be
	# 				activated
	# Return:		None
	def selectDigit(self, digit):
		
		self.allPinsOff()

		if digit == 0:
			self.__D0.off()	
		if digit == 1:
			self.__D0.off()
		if digit == 2:
			self.__D0.off()
		if digit == 3:
			self.__D0.off()

	# Name:			allOn
	# Description:	PinSevenSegment class method which turns all digits and all segments on for
	#				testing purposes
	# Parameters:	None
	# Return:		None
	def allOn(self):
		
		self.__D0.off()
		self.__D1.off()
		self.__D2.off()
		self.__D3.off()
		
		self.__A.on()
		self.__B.on()
		self.__C.on()
		self.__D.on()
		self.__E.on()
		self.__F.on()
		self.__G.on()
		self.__DP.on()

	# Name:			allPinsOff
	# Description:	PinSevenSegment class method which takes in strings corresponding to the voltage value
	# 				to be displayed on the seven segment and the proximity threshold which is only
	# 				included so as to make the interface compatible with the class method's use
	# 				elsewhere in the program and which is discarded
	# Parameters:	Takes in two strings, the voltage value and the proximity value, both as strings
	# Return:		None
	def displayNumber(self, voltage, proximity):

		voltageSize = len(voltage)
		
		if voltageSize > 0:
			decimal = False

			# the string is reversed so that the least significant digits are 
			# at the front of the string  
			voltageString = "".join(reversed(voltage))
			
			digitSelect = 0
			counter = 0

			# iterates through the string and writes it, digit by digit, to the
			# seven segment display
			while counter < voltageSize and digitSelect < self.__MAX_DIGITS:
			
				# if a decimal point is encountered, a bool is set and the position in
				# the string is advanced past it
				if voltageString[counter] == self.__PERIOD_CHAR:
					decimal = True 
					counter += 1

				# the previous number is zeroed out and the appropriate digit on the 
				# seven segment is selected
				self.zero()
				self.selectDigit(digitSelect)
				
				# the current digit in the string is written to the seven segment display,
				# a sleep is used to make the digit visible to the eye
				self.numberSelect(voltageString[counter], decimal)
				sleep(self.__DISPLAY_DELAY)

				decimal = False
				counter += 1
				digitSelect += 1

	# Name:			numberSelect
	# Description:	PinSevenSegment class method takes in a char which is the number to be written
	# 				to the digit and a bool corresponding to whether or not the decimal point will
	# 				be written, and inspects the number to determine which segments to write high
	# Parameters:	Takes in the number to write to the digit as a char and a bool which determines
	# 				whether or not the decimal point will be written
	# Return:		None
	def numberSelect(self, number, decimalPoint):
		
		if not number == '1' and not number == '4':
			self.__A.on()
		if not number == '5' and not number == '6':
			self.__B.on()
		if not number == '2':
			self.__C.on()
		if not number == '1' and not number == '4' and not number == '7' and not number == '9':
			self.__D.on()
		if number == '0' or number == '2' or number == '6' or number == '8':
			self.__E.on()
		if not number == '1' and not number == '2' and not number == '3' and not number == '7':
			self.__F.on()
		if not number == '0' and not number == '1' and not number == '7':
			self.__G.on()
			
		if decimalPoint:
			self.__DP.on()
