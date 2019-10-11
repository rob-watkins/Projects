# Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 
# Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins
# Subsystem:	GUI
# File name:	quaternion_loader.py 
# Description:	QuaternionManager class implementation	
#				QuaternionManager class reads in quaternion constraints for each sensor from
#				a locally saved file and generates an XML launch file that wraps these constraints
#				into individual nodes to be published to ROSmaster

from pathlib import Path

class QuaternionManager:

	column, row = 6, 12;

	def __init__( self ):
		# initialize a 2D list( 6 x 12 )
		self.quatList = [[0 for x in range(self.column)] for y in range(self.row)]

	# Name:			launchWriter
	# Description:	QuaternionManager class method which creates an XML launch file from the data
	#				within the quatList 2D list containing sensor positional data. This method creates
	#				the launch file and overwrites any currently stored launch file. Each sensor is sequenced 
	#				and all supplementary parameters vital for ROSmaster to use the sensor offset data such as 
	#				package labels, method tags, and destination paths are also written into each node.
	# Parameters:	None
	# Return:		None
	def launchWriter( self ):
		launchFile = open("quaternion.launch","w")

		# opening xml tag
		launchFile.write( "<launch> \n \n" )

		for x in range(0, 12):
			# node identifying comment
			launchFile.write( "\t <!-- STATIC TRANSFORM FROM DRONE BODY TO ToF SENSOR " 
																+ str( x + 1) + "--> \n" )
			# node package tag
			launchFile.write( "\t <node pkg=\"tf\" \n" )

			# node method tag
			launchFile.write( "\t \t   type=\"static_transform_publisher\" \n" )

			# sensor identity tag
			launchFile.write( "\t \t   name=\"drone_ToF_sensor_" + str( x + 1 ) + "\" \n" )

			# sensor offset and transform parameters and destination paths
			launchFile.write( "\t \t   args=\"" + str(self.quatList[x][0]) + " "
				+ str(self.quatList[x][1]) + " " + str(self.quatList[x][2]) + " "
				+ str(self.quatList[x][3]) + " " + str(self.quatList[x][4]) + " "
				+ str(self.quatList[x][5]) + " /firefly/base_link /quat/tof_tf_" + str( x + 1 ) + " 100\"/> \n \n" )

		#closing xml tag
		launchFile.write( "</launch>" )

		launchFile.close()

	# Name:			loadValues
	# Description:	QuaternionManager class method which takes in a the file name for stored 
	#				quaternion data and loads it into quatList, the 2D class list. The file must be
	#				comma delimitted with 6 integers or floats per line( each line represents a sensor 
	#				and each number represents x, y, z offsets and roll, pitch, yaw transforms respectively).
	#				The method tests that a file does exist and calls the launchWriter method after values
	#				have been loaded in. 
	# Parameters:	Takes in a string which represents the name of the file containing quaternion data.
	# Return:		None
	def loadValues( self, fileName ):
		quatIndex = 0

		# set to path for validity check
		fName = Path(fileName)

		if fName.is_file():
			quatFile = open( fileName, "r" )

			for line in quatFile:
				line = line.strip('\n')	# strip newline
				parameters = line.split(", ") # split values by comma

				# each value is written as a separate list item
				for param in range(self.column):
					self.quatList[quatIndex][param] = parameters[param]

				quatIndex += 1

			quatFile.close()

			try:
				self.launchWriter() # write the xml launch file
			except:
				print("There was a problem writing the launch file.")

		else:
			print("File name invalid")
