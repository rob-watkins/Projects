Project: 		CS426 Spring 2018, Team #23: SkyWarden Senior Project, Aerial Drone Notification System (ADNS) 

Team:			Rony Calderon, Bryan Kline, Jia Li, Robert Watkins

Subsystem:	    	Ground Base Unit and GUI

File name:	    	readme 

Description:    	Provides the commands to initialize the subsystem running on the Raspberry Pi in the ground base
                	unit for the headless versions of ADNS

To run the system, in the "Headless ADNS" directory:

    python3 HeadlessGroundBaseUnit.py


System initialization:

All of the following commands have been carried out on the Raspberry Pi prior to shipping the system, however 
the commands are provided here for reference. 


Serial port:
------------

In order to set up the pyserial libraries that the system depends on to read in values from the serial port,
the following must be carried out once, if the libraries do not already exist.

    pip install pyserial

This command downloads pyserial-3.4, to build and install the library in the pyserial-3.4 folder:

	python setup.py build
	sudo python setup.py install		

The following is not necessary as the system issues the appropriate system calls, however for reference
the serial port permissions are set with the following command:

	sudo chmod 777 /dev/ttyACM0

The pyserial-3.4 must be in the same directory as the source code.

ROS:
----

The appropriate ROS libraries must be downloaded and installed prior to running the system:

    sudo apt-get -y install python3-pip
	sudo apt-get update
	sudo pip3 install PyYAML
	sudo pip3 install rospkg
	sudo pip3 install catkin_pkg
	sudo pip3 install tf
	sudo pip3 install geometry

In order to set the correct IP addresses for the Raspberry Pi and the ROS master node, the following
commands must be issued prior to running the system:

On the ROS master node, where <MASTER_IP> is the IP address of the ROS master node:
	
    export ROS_MASTER_URI=http://<MASTER_IP>:11311
	export ROS_IP=<MASTER_IP> 

On the Raspberry Pi, where <RASPBERRY_PI_IP> is the IP address of the Raspberry Pi:
	export ROS_MASTER_URI=http://<RASPBERRY_PI_IP>:11311
	export ROS_IP=<RASPBERRY_PI_IP>


 

