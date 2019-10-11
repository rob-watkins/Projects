
# Project: 		CS426 Spring 2018, Team #23: SkyWarden ADNS Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	GenericLEDs.py 
# Description:	GenericLED class implementation
#				GenericLED class accepts a hexadecimal value from the ground base unit driver which corresponds
# 				to the generic alert flags which are published from the ROS master node, and uses that value
# 				to drive the pins on the I2C bus on the MPR121 which in turn drive eight LEDs on the ground unit
# 				which correspond to the generic alerts which can be added to the system.  All bus accesses are
# 				wrapped in try except blocks which allow the I2C to retry chip address acquisition in the event
#				that the bus loses its address

# Hardware:
# I2C Bus:							MPR121

import subprocess
import smbus
from time import sleep

# MPR121 I2C bus constants for initializing the chip
GPIO_CONTROL_ZERO = 0x73
GPIO_CONTROL_ONE = 0x74
GPIO_DATA = 0x75
GPIO_DIRECTION = 0x76
GPIO_ENABLE = 0x77

ALL_PINS = 0xFF
ZERO_PINS = 0x00
BUS_NUMBER = 1

# MPR121 chip addresses
CHIP_ADDRESS = 0x5A

# creation of an SMBus object from the smbus library for the use of the I2C bus
bus = smbus.SMBus(BUS_NUMBER)

class GenericLEDs:

	# Name:			Default constructor
	# Description:	GenericLEDs class default constructor which initializes the MP121 I2C bus
	# 				which drives the LEDs for the eight generic alerts on the ground base unit
	# Parameters:	None
	# Return:		None
	def __init__(self):

		# initialization of I2C bus for the generic LEDs
		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_CONTROL_ZERO, ALL_PINS)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])	

		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_CONTROL_ONE, ALL_PINS)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])	

		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_DIRECTION, ALL_PINS)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])	

		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_ENABLE, ALL_PINS)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])

		self.clear()

	# Name:			clear
	# Description:	GenericLEDs class method writes the hexadecimal value 0x00 to the I2C bus
	# 				which sets all pins to low, thereby turning off all generic LEDs on the ground
	# 				unit 
	# Parameters:	None
	# Return:		None
	def clear(self):

		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_DATA, ZERO_PINS)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])

	# Name:			setLEDs
	# Description:	GenericLEDs class method which takes in a hexadecimal value which is built outside of the
	# 				class and which codes the state of each of the eight generic alerts, the value is written to
	# 				the bus, which in turns drives the eight LEDs which act as the generic alerts on the ground
	# 				unit
	# Parameters:	Takes in a hexadecimal value which corresponds to the eight generic alerts where
	# 				a one is high and a zero is low
	# Return:		None	
	def setLEDs(self, hexValue):
			
		try:
			bus.write_byte_data(CHIP_ADDRESS, GPIO_DATA, hexValue)
		except IOError:
			subprocess.call(['i2cdetect', '-y', '1'])
