
# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	Ground Base Unit
# File name:	GroundUnitDriver.py 
# Description:	Main driver for ground unit
#				Main driver which creates various objects, processes, pipes, and threads
#				which read in values from the serial port from the drone, send out values
#				to ROS nodes and the GUI, and pipe in values from the GUI, which control 
#				the hardware on the ground base unit 

from time import sleep
from threading import Thread
from multiprocessing import Process, Pipe
from Parser import Parser
from GUISubsystem import windowProcess
from ROSNodeManager import ROSNodeManager
from GenericLEDs import GenericLEDs
from quaternion_loader import QuaternionManager
from ShiftSevenSegment import ShiftSevenSegment
from PinSevenSegment import PinSevenSegment
from gpiozero import LED, Button
import subprocess

# main loop, pin number, thread, and process constants and variables

SPEAKER_PIN = 17
VOLTAGE_LED_PIN = 12
PROXIMITY_LED_PIN = 25
VOLTAGE_BUTTON_LED_PIN = 8
PROXIMITY_BUTTON_LED_PIN = 7
VOLTAGE_RESET_PIN = 10
PROXIMITY_RESET_PIN = 9
GENERIC_SWITCH_0_PIN = 4 
GENERIC_SWITCH_1_PIN = 14
GENERIC_SWITCH_2_PIN = 15
GENERIC_SWITCH_3_PIN = 18

INITIALIZATION_ONE = 1
BAUD_RATE = 1000000
TIME_OUT = 0.5
PARSE_CONTROL = 1
INFINITY = 99999
SWITCH_COUNT = 4
BIT = 0
BYTE = 1

voltage = "00.00"
threshold = '1000'
voltThresh = '0.000'
proxThresh = '1000'
shiftSegment = False
voltageAlert = False
proximityAlert = False
voltageReset = True
proximityReset = True
shiftSegment = True
LEDHex = 0x00
switchList = None
segList = [voltage, proxThresh]
alertList = [voltageAlert, proximityAlert]


# gpiozero LED and Button object declarations which tie directly into GPIO
# pins on the Raspberry Pi, the constants taken into the constructors are
# the physical pins on the microcontroller
speaker = LED(SPEAKER_PIN)
voltageLED = LED(VOLTAGE_LED_PIN)
proximityLED = LED(PROXIMITY_LED_PIN)
voltageButtonLED = LED(VOLTAGE_BUTTON_LED_PIN)
proximityButtonLED = LED(PROXIMITY_BUTTON_LED_PIN)
voltageResetButton = Button(VOLTAGE_RESET_PIN, True, 1.0)
proximityResetButton = Button(PROXIMITY_RESET_PIN, True, 1.0)
switchList = [Button(GENERIC_SWITCH_0_PIN, True), Button(GENERIC_SWITCH_1_PIN, True), 
	Button(GENERIC_SWITCH_2_PIN, True), Button(GENERIC_SWITCH_3_PIN, True)]

# class object creation for the Parser, ROSNodeManager, SevenSegment, and
# ComponentManager objects used in the main loop, threads, and processes
parser = Parser(BAUD_RATE, TIME_OUT, BYTE)
nodeManager = ROSNodeManager()

if shiftSegment:
sevenSegment = ShiftSevenSegment()
else:
	sevenSegment = PinSevenSegment()
		
genericLED = GenericLEDs()

# creation of logical pipes which move data into and out of the GUI, and
# between processes within the main driver for the speaker and seven segments 
parentOutStream, childOutStream = Pipe()
parentInStream,childInStream = Pipe()
speakerPipeIn, speakerPipeOut = Pipe()
sevenSegPipeIn, sevenSegPipeOut = Pipe()

# sets up two logical pipes to and from the GUI in order to send out voltage values
# and read in proximity threshold values
streamPipe = Process(target=windowProcess, args=(childOutStream, childInStream,))
streamPipe.start()


# Name:			getGenericSwitches
# Description:	Free function which takes in a list of Button objects, iterates through
# 				it and produces a list of flags holding the states of the Button objects
# 				which is used to send to the ROSNodeManager object in order to publish
# 				the states of the toggle switches
# Parameters:	Takes in a list of Button objects from gpiozero which correspond to the
# 				states of the toggle switches for generic alerts on the ground unit
# Return:		Returns a list of ints which hold flags for each of the states of the 
# 				Button objects
def getGenericSwitches(buttonList):
	
	localSwitchList = [0, 0, 0, 0]

	index = 0
	while index < SWITCH_COUNT:
		if buttonList[index].is_pressed:
			localSwitchList[index] = 1
		index += 1

	return localSwitchList

# Name:			updateSevenSegment
# Description:	Free function used to establish a separate process which continually 
# 				polls a Pipe object to receive voltage and proximity threshold data 
# 				which are then sent to the SevenSegment object to display the values 
# 				on the hardware on the ground unit
# Parameters:	Takes in a logical pipe which is used stream in values for voltage and the
# 				proximity threshold from the main loop into the process which updates
# 				the seven segments
# Return:		None
def updateSevenSegment(pipeConnection):
	
	sleep(1)

	valList = ["00.00", "1000"]
	
	while True:
		# if data is available from the main loop, store it in a list
		if pipeConnection.poll():
			valList = pipeConnection.recv()

		# if there are values for both voltage and proximity threshold, write them to the
		# seven segment displays
		if valList[0] != None and valList[1] != None:
			sevenSegment.displayNumber(valList[0], valList[1])

# sets up a logical pipe between the updateSevenSegment function and the main loop
# so that seven segment displays are updated in parallel with the main loop in 
# order to reduce latency and starts the process
sevenSegPipe = Process(target=updateSevenSegment, args=(sevenSegPipeIn,))
sevenSegPipe.start()

# Name:			resetProxMinCounter
# Description:	Free function which, running as a separate thread, declares a global
# 				value for the minimum proximity value encountered in a certain period
# 				of time, and then continually resets the minimum to a constant standing
# 				in for infinity so that a new minimum value can be established in order
# 				to refresh the minimum proximity value in the main loop which is used to
# 				issue an alert for proximity
# Parameters:	None
# Return:		None
def resetProxMinCounter():

	global proxMin
	proxMin = INFINITY

	while True:
		sleep(0.1)
		proxMin = INFINITY

# sets up and starts the thread which continually updates the global proximity minimum
resetProxMinThread = Thread(target=resetProxMinCounter)	
resetProxMinThread.start()

# Name:			updateSpeaker
# Description:	Free function which creates a logical pipe between the main loop and 
# 				the function running concurrently in a separate thread which sends in
# 				bools corresponding to whether an alert has been issue for voltage and 
# 				proximity and the speaker is activated if either is true
# Parameters:	Takes in a Pipe object which sends in bools for the voltage and proximity
# 				alerts which determine whether or not to activate the speaker
# Return:		None
def updateSpeaker(pipeConnection):

	sleep(1)
	alertFlags = [0, 0]

	while True:

		# if alert flags are available, get them  	
		if pipeConnection.poll():
			alertFlags = pipeConnection.recv()

		# if either flag is high, play the speaker for one cycle
		#for x in range( 0, 15 ):
		if alertFlags[0] or alertFlags[1]:
			speaker.on()
			sleep(0.00099)
			speaker.off()
			sleep(0.00099)

# sets up and starts the thread which activates the speaker if voltage or proximity data
# come into the system below either of their thresholds
speakerThread = Process(target=updateSpeaker, args=(speakerPipeIn,))
speakerThread.start()

# Name:			debounceButtons
# Description:	Free function which runs concurrently with the main loop in a separate thread
# 				which debounces the voltage and proximity alert reset buttons
# Parameters:	None
# Return:		None
def debounceButtons():

	global voltageReset
	voltageReset = False

	global proximityReset
	proximityReset = False

	while True:

		if voltageResetButton.is_pressed:
			if voltageReset:
				voltageReset = False
			else:
				voltageReset = True
				
			sleep(0.25)

		if proximityResetButton.is_pressed:
			if proximityReset:
				proximityReset = False
			else:
				proximityReset = True
				
			sleep(0.25)

# sets up and starts the thread which debounces the reset buttons
buttonThread = Thread(target=debounceButtons)
buttonThread.start()

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

# LEDs are initialized to off
voltageLED.off()
proximityLED.off()
voltageButtonLED.off()
proximityButtonLED.off()
speaker.off()

sleep(INITIALIZATION_ONE)

# the main loop which continually sends out alerts through the pipe to the speaker, seven
# segment displays, polls the GUI pipe for threshold data, formats that data if present,
# sends data to the ROSNodeManager object to publish and subscribe to various ROS topics,
# sends voltage data to the GUI, formats the voltage data, and activates the alert LEDs if
# a threshold is out of range
while True:

	# voltage and proximity alert bools are piped into the speaker process
	alertList[0] = voltageAlert
	alertList[1] = proximityAlert
	speakerPipeOut.send(alertList)

	# voltage and proximity threshold values are piped into the seven segment display process
	segList[0] = voltage
	segList[1] = proxThresh
	sevenSegPipeOut.send(segList)

	# the pipe from the GUI is polled for threshold data, if present then the data is inspected
	# to see if it is a voltage or a proximity threshold, if a proximity threshold then the value
	# is padded if necessary for output on the seven segment displays
	if parentInStream.poll():
		threshold = parser.parsePipeInput(parentInStream.recv())

		if threshold != None and threshold[len(threshold) - 1] == 'P':
			proxThresh = threshold[:-1]
			if len(proxThresh) == 3:
				proxThresh = '0' + proxThresh
			if len(proxThresh) == 2:
				proxThresh = '00' + proxThresh
			if len(proxThresh) == 1:
				proxThresh = '000' + proxThresh

		elif threshold != None and threshold[len(threshold) - 1] == 'V':
			voltThresh = threshold[:-1]

	# the Parser class object method to get values from the serial port is invoked to bring
	# data in from the drone
	value = parser.getSerialInput() 

	# voltage and proximity values are publish to ROS, and generic switch and LED alerts are
	# published and subscribed to  
	
	if value != None and len(value) > 1 and (value[0] == 'v' or ((value[0] >= 'a' and value[0] <= 'n') and value[1].isnumeric() and int(value[1]) < 2800)):
		nodeManager.publishSensorValue(value)
	
	genericList = getGenericSwitches(switchList)
	nodeManager.publishGenericSwitches(genericList)
	LEDHex = nodeManager.subscribeGenericLEDs()
	genericLED.setLEDs(LEDHex)
	
	# the voltage data is sent out through the pipe to the GUI
	if value != None and len(value) > 1 and value[0] == 'v':
		parentOutStream.send(value)

	# the value from the serial port, if a voltage, is set to the appropriate variable and 
	# if necessary it is padded for display on the seven segment 
	if value != None and len(value) > 1 and value[0] == 'v' and threshold != None:
		voltage = value[1]	
		if len(voltage) == 4:
			voltage = '0' + voltage
		if len(voltage) == 3:
			voltage = '00' + voltage
		if len(voltage) == 2:
			voltage = '000' + voltage
		if len(voltage) == 1:
			voltage = '000.' + voltage

	# else if the value from the serial port is a proximity, it is tested against the lowest 
	# so far encountered since the previous reset
	elif value != None and len(value) > 1 and value[0] >= 'a' and value[0] <= 'n':
		if value[1].isnumeric() and int(value[1]) < 2800 and int(value[1]) < proxMin:
			proxMin = int(value[1])
		

	# the voltage and proximity values that have come in are tested against thresholds and if outside 
	# their acceptable ranges their alerts are issued

	if not voltageReset and voltage != None and voltThresh != None and float(voltage) < float(voltThresh):
		voltageAlert = True
		voltageLED.on()
		voltageButtonLED.on()
	else:
		voltageAlert = False
		voltageLED.off()
		voltageButtonLED.off()

	if not proximityReset and proxThresh != None and int(proxMin) < int(proxThresh) and proxMin != INFINITY:
		proximityAlert = True
		proximityLED.on()
		proximityButtonLED.on()
	else:
		proximityAlert = False
		proximityLED.off()
		proximityButtonLED.off()
