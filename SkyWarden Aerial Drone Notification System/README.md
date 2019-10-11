# SkyWarden Aerial Drone Notification System

Name: SkyWarden Aerial Drone Notification System


Authors: Rony Calderon, Bryan Kline, Jia Li, Robert Watkins


Dates:  August 2017 - May 2018


The "SkyWarden: Aerial Drone Notification System" project is an alert and notification system for aerial drones created as 
a senior project for the Autonomous Robots Lab (http://www.autonomousrobotslab.com/) at the University of Nevada, Reno over the
2017-2018 year.

The system consists of three main subsystems: an on-board drone subsystem, a ground base unit subsystem,
and a GUI subsystem.  The on-board subsystem is composed of a voltage sensor and a LiDAR sensor array which are polled by
a microcontroller and transmitted wirelessly to the ground base unit.  The code for the on-board subsystem is contained in the "On-Board Subsystem" directory, along with the documentation for the on-board subsystem which contains the wiring schematic, and is written in
ANSI C and C++. 

The ground base unit subsystem is composed of a Raspberry Pi which controls a number of LEDs, seven segment displays, buttons, switches, 
and a speaker, which continually receives the voltage and proximiyt data from the drone, displays it on the device, and issues visual and auditory alerts when any data falls below user-defined thresholds.  The code for the ground base unit subsystem is contained in the "Ground Base Unit and GUI Subsystems" directory, which contains subdirectories for the ground unit source code, datasheets and wiring diagrams
for the hardware used in the construction of the ground unit, and the documentation for the ground base unit.  The code for the ground base unit is written in Python.

The Raspberry Pi also runs a GUI which the user may use in order to view the data graphically and to set the voltage and proximity thresholds.  The data is also streamed over the network through the Robot Operating System, ROS, which researchers use to control and visualize the drone.  The code and documentation for the GUI subsystem is located in the "Ground Base Unit and GUI Subsystems", and is written in Python.

