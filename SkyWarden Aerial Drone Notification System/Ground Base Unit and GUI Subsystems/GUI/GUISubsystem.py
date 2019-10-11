## Program Information ########################################################
###
# @file ADNS_Main_GUI.py
#
# @brief Driver program that executes the main GUI engine that allows 
#        user interaction with the Aerial Drone notification system.
# 
# @details Allows efficient quaternion and threshold settings
#
# @version 1.10
#          Rony Calderon, Bryan Kline, Jia Li, and Robert Watkins (Feb. 2018)
#          Implemented the sensor offset, threshold, and quaternion data 
#
#          1.00 
#          Rony Calderon, Bryan Kline, Jia Li, and Robert Watkins (Nov. 2017)
#          Original code 
#
# @Note N/A

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, Gdk
from multiprocessing import Process, Pipe
from threading import Thread
from time import time, sleep
import numpy
from matplotlib import pyplot


class MyWindow(Gtk.Window):

    __sendChild = None   
    __receiveVoltageChild = None
    __receiveFlagChild = None 
    __alertIssued = False
    __startTime = 0.0

    def __init__(self):

        WINDOW_HEIGHT = 150
        WINDOW_WIDTH = 100

        MENU_COLUMN = 0
        MENU_ROW = 0

        SPACER_COLUMN = 0
        SPACER_ONE_ROW = 1
        SPACER_TWO_ROW = 3
        SPACER_THREE_ROW = 6

        VOLTAGE_ROW = 2
        VOLTAGE_LABEL_COLUMN = 0
        VOLTAGE_COLUMN = 1
        VOLTAGE_UNIT_COLUMN = 2

        VOLTAGE_THRESHOLD_ROW = 4
        VOLTAGE_THRESHOLD_LABEL_COLUMN = 0
        VOLTAGE_THRESHOLD_COLUMN = 1
        VOLTAGE_THRESHOLD_UNIT_COLUMN = 2
        VOLTAGE_THRESHOLD_BUTTON_COLUMN = 3

        PROXIMITY_THRESHOLD_ROW = 5
        PROXIMITY_THRESHOLD_LABEL_COLUMN = 0
        PROXIMITY_THRESHOLD_COLUMN = 1
        PROXIMITY_THRESHOLD_UNIT_COLUMN = 2
        PROXIMITY_THRESHOLD_BUTTON_COLUMN = 3
        
        BUTTON_ROW = 7
        SAVE_BUTTON_COLUMN = 0
        PLOT_BUTTON_COLUMN = 1
        DEFAULT_DIMENSION = 1

        logFile = open("voltage.log", "w")
        logFile.close()
        logFile = open("voltage.plot", "w")
        logFile.close()

        self.__startTime = time()

        Gtk.Window.__init__(self, title = "Aerial Drone Notification System")
        self.set_size_request(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.grid = Gtk.Grid()

        color = Gdk.color_parse("LimeGreen")
        rgbb = Gdk.RGBA.from_color(color)
        self.grid.override_background_color(0, rgbb)
        self.add(self.grid)

        systemMenu = Gtk.MenuBar()
        systemMenu.set_hexpand(True)
        self.grid.attach(systemMenu, MENU_COLUMN, MENU_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        systemMenuLabel = Gtk.MenuItem(label="System")
        systemMenu.append(systemMenuLabel)
        menuList = Gtk.Menu()
        systemMenuLabel.set_submenu(menuList)

        sensorOffsetsMenuItem = Gtk.MenuItem(label="Set Sensor Offsets")
        menuList.append(sensorOffsetsMenuItem)
        sensorOffsetsMenuItem.connect('activate', self.set_quaternion)

        savePlotMenuItem = Gtk.MenuItem(label="Capture voltage values")
        menuList.append(savePlotMenuItem)
        savePlotMenuItem.connect('activate', self.on_saveButton_clicked)

        savePlotMenuItem = Gtk.MenuItem(label="Plot voltage values")
        menuList.append(savePlotMenuItem)
        savePlotMenuItem.connect('activate', self.on_plotButton_clicked)
        
        exitMenuItem = Gtk.MenuItem(label="Exit")
        menuList.append(exitMenuItem)
        exitMenuItem.connect('activate', Gtk.main_quit)

        self.dummyLabel1 = Gtk.Label(" ")
        self.grid.attach(self.dummyLabel1, SPACER_COLUMN, SPACER_ONE_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.voltageLabel = Gtk.Label("Voltage: ")
        self.voltage = Gtk.Label(" ") 
        self.voltageUnitsLabel = Gtk.Label(" V ")
        self.grid.attach(self.voltageLabel, VOLTAGE_LABEL_COLUMN, VOLTAGE_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.voltage, VOLTAGE_COLUMN, VOLTAGE_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.voltageUnitsLabel, VOLTAGE_UNIT_COLUMN, VOLTAGE_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.dummyLabel2 = Gtk.Label(" ")
        self.grid.attach(self.dummyLabel2, SPACER_COLUMN, SPACER_TWO_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.voltageThresholdLabel = Gtk.Label("Voltage Threshold: ")
        self.voltageThresholdEntry = Gtk.Entry() 
        self.voltageThresholdUnitsLabel = Gtk.Label(" V ")
        self.voltageThresholdButton = Gtk.Button(label="Set Voltage Threshold")
        self.grid.attach(self.voltageThresholdLabel, VOLTAGE_THRESHOLD_LABEL_COLUMN, VOLTAGE_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.voltageThresholdEntry, VOLTAGE_THRESHOLD_COLUMN, VOLTAGE_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.voltageThresholdUnitsLabel, VOLTAGE_THRESHOLD_UNIT_COLUMN, VOLTAGE_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.voltageThresholdButton.connect("clicked", self.on_voltageThreshold_clicked)
        self.grid.attach(self.voltageThresholdButton, VOLTAGE_THRESHOLD_BUTTON_COLUMN, VOLTAGE_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.proximityThresholdLabel = Gtk.Label("Proximity Threshold: ")
        self.proximityThresholdEntry = Gtk.Entry()
        self.proximityThresholdUnitsLabel = Gtk.Label(" mm ")
        self.proximityThresholdButton = Gtk.Button(label="Set Proximity Threshold")
        self.grid.attach(self.proximityThresholdLabel, PROXIMITY_THRESHOLD_LABEL_COLUMN, PROXIMITY_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.proximityThresholdEntry, PROXIMITY_THRESHOLD_COLUMN, PROXIMITY_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.grid.attach(self.proximityThresholdUnitsLabel, PROXIMITY_THRESHOLD_UNIT_COLUMN, PROXIMITY_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
        self.proximityThresholdButton.connect("clicked", self.on_proximityThreshold_clicked)
        self.grid.attach(self.proximityThresholdButton, PROXIMITY_THRESHOLD_BUTTON_COLUMN, PROXIMITY_THRESHOLD_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.dummyLabel3 = Gtk.Label(" ")
        self.grid.attach(self.dummyLabel3, SPACER_COLUMN, SPACER_THREE_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.saveButton = Gtk.Button(label="Capture Voltage")
        self.saveButton.connect("clicked", self.on_saveButton_clicked)
        self.grid.attach(self.saveButton, SAVE_BUTTON_COLUMN, BUTTON_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        self.plotButton = Gtk.Button(label="Plot Voltage")
        self.plotButton.connect("clicked", self.on_plotButton_clicked)
        self.grid.attach(self.plotButton, PLOT_BUTTON_COLUMN, BUTTON_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        settings = Gtk.Settings.get_default()
        settings.set_property("gtk-theme-name", "Numix")
        settings.set_property("gtk-icon-theme-name", "Numix")
        settings.set_property("gtk-application-prefer-dark-theme", False)

        self.update = Thread(target=self.updateWindow)
        self.update.setDaemon(True)
        self.update.start()        

    def updateWindow(self):

        counter = 0

        while True:
            sleep(0.001)
            voltageText = self.__receiveVoltageChild.recv()
            if voltageText != None and len(voltageText) > 1:
	            voltageText = str(voltageText[1])
	            GObject.idle_add(self.voltage.set_text, voltageText, priority=GObject.PRIORITY_DEFAULT)
	            if counter > 1000:	
	                self.logVoltage(voltageText)
	                counter = 0
	            else:
	         	    counter += 1

            alert = self.__receiveFlagChild.recv()

            if alert == 'A' and not self.__alertIssued:
                self.changeBackgroundColor("red")
                self.__alertIssued = True
            elif alert == 'N' and self.__alertIssued:
                self.changeBackgroundColor("green")
                self.__alertIssued = False

            

    def changeBackgroundColor(self, color):
    
        if color == "red":
            color = Gdk.color_parse("firebrick1")
        elif color == "green":   
            color = Gdk.color_parse("LimeGreen") 

        rgbHex = Gdk.RGBA.from_color(color)
        self.grid.override_background_color(0, rgbHex)        

    def logVoltage(self, voltage):
    
        logFile = open("voltage.log", "a")
        timeStamp = str(time() - self.__startTime)
        logFile.write(voltage + " " + timeStamp + "\n")
        logFile.close()

    def on_saveButton_clicked(self, widget):    
       	
       	plotFile = open("voltage.plot", "w")
       	logFile = open("voltage.log", "r")

       	for line in logFile:
       		plotFile.write(line)

       	logFile.close()
       	plotFile.close()	

    def on_plotButton_clicked(self, widget):
        
        voltageList = []
        timeList = []

        file = open("voltage.plot", "r")

        for line in file: 
            line = line.split(" ")
            voltageList.append(float(line[0]))
            timeStamp = float(line[1][:5])
            timeList.append(timeStamp)

        file.close() 
        
        pyplot.title("Voltage vs. Time") 
        pyplot.xlabel("seconds") 
        pyplot.ylabel("volts") 
        pyplot.plot(timeList, voltageList) 
        pyplot.show()   

    def on_voltageThreshold_clicked(self, widget): 
        voltageThreshold = self.voltageThresholdEntry.get_text()
        if voltageThreshold != None:
            voltageThreshold = voltageThreshold + "V"
            self.__sendChild.send(voltageThreshold)

    def on_proximityThreshold_clicked(self, widget):
        proximityThreshold = self.proximityThresholdEntry.get_text()
        if proximityThreshold != None:
            proximityThreshold = proximityThreshold + "P"
            self.__sendChild.send(proximityThreshold)

    def set_quaternion(self, widget): 
        QuaternionWin = QuaternionWindow() 
        QuaternionWin.show_all() 

    def setUpSendPipe(self, child):
        self.__sendChild = child 

    def setUpVoltageReceivePipe(self, child):
        self.__receiveVoltageChild = child 

    def setUpFlagReceivePipe(self, child):
        self.__receiveFlagChild = child     


class QuaternionWindow( Gtk.Window ): 

    __MAX_ROW = 14
    __MAX_COLUMN = 6
    __NEW_LINE = 5

    __quaternionData = []
    __sensorLabels = []
    __sensorEntries = []
    __sensorFieldLabels = []

    __grid = None

    def __init__(self): 

        SAVE_BUTTON_COLUMN = 0
        SAVE_BUTTON_ROW = 15
        DEFAULT_DIMENSION = 1

        Gtk.Window.__init__(self, title="Set Sensor Offsets")

        self.__grid = Gtk.Grid()

        color = Gdk.color_parse("LimeGreen")
        rgbb = Gdk.RGBA.from_color(color)
        self.__grid.override_background_color(0, rgbb) 
        self.add(self.__grid) 

        self.initializeQuaternionData()
        self.buildFields()
        self.readConfigFile()
        self.populateFields()
       
        self.saveButton = Gtk.Button(label="Save")
        self.saveButton.connect("clicked", self.on_save_clicked)
        self.__grid.attach(self.saveButton, SAVE_BUTTON_COLUMN, SAVE_BUTTON_ROW, DEFAULT_DIMENSION, DEFAULT_DIMENSION)

        settings = Gtk.Settings.get_default()
        settings.set_property("gtk-theme-name", "Numix")
        settings.set_property("gtk-icon-theme-name", "Numix")
        settings.set_property("gtk-application-prefer-dark-theme", False)

    def initializeQuaternionData(self):
    
        self.__quaternionData = [[0 for column in range(self.__MAX_COLUMN)] for row in range(self.__MAX_ROW)]

    def buildFields(self):

        DEFAULT_DIMENSION = 1

        fieldLables = ["x (m):", "y (m):", "z (m):", "pitch (rad):", "roll (rad):", "yaw (rad):"]

        self.__sensorLabels = [Gtk.Label("Sensor " + str(row) + ": ") for row in range(self.__MAX_ROW)]
        self.__sensorEntries = [[Gtk.Entry() for column in range(self.__MAX_COLUMN)] for row in range(self.__MAX_ROW)] 
        self.__sensorFieldLabels = [[Gtk.Label(fieldLables[column]) for column in range(self.__MAX_COLUMN)] for row in range(self.__MAX_ROW)] 

        for row in range(0, self.__MAX_ROW):
            self.__grid.attach(self.__sensorLabels[row], 0, row, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
            for column in range(0, self.__MAX_COLUMN):
                    self.__grid.attach(self.__sensorFieldLabels[row][column], ((column * 2) + 1), row, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
                    self.__grid.attach(self.__sensorEntries[row][column], ((column * 2) + 2), row, DEFAULT_DIMENSION, DEFAULT_DIMENSION)
              
    def populateFields(self):
    
        for row in range(0, self.__MAX_ROW):
            for column in range(0, self.__MAX_COLUMN): 
                self.__sensorEntries[row][column].set_text(str(self.__quaternionData[row][column]))                

    def readConfigFile(self):

        with open("sensorOffsets.config", "r") as launchFile:

            row = 0 
            for line in launchFile:
                column = 0 
                parsedLine = line.split(", ")
                for word in parsedLine:
                    if word[len(word) - 1] == '\n':
                        word = word[:-1]
                    self.__quaternionData[row][column] = float(word)  
                    column += 1
                row += 1

        launchFile.close()    

    def on_save_clicked(self, widget):

        self.getFieldData()

        launchFile = open("sensorOffsets.config","w")

        for row in range(0, self.__MAX_ROW):
            for column in range(0, self.__MAX_COLUMN):
                launchFile.write(str(self.__quaternionData[row][column]))
                if(column == self.__NEW_LINE):
                    launchFile.write("\n")
                else:
                    launchFile.write(", ")

        launchFile.close()

    def getFieldData(self):
    
        for row in range(0, self.__MAX_ROW):
            for column in range(0, self.__MAX_COLUMN):
                self.__quaternionData[row][column] = self.__sensorEntries[row][column].get_text()


def windowProcess(rec_voltage_child, rec_flags_child, send_child):
            
    window = MyWindow()

    window.setUpVoltageReceivePipe(rec_voltage_child)
    window.setUpFlagReceivePipe(rec_flags_child)
    window.setUpSendPipe(send_child)

    GObject.threads_init()
    window.connect("delete-event", Gtk.main_quit)
    window.show_all()
    Gtk.main()
