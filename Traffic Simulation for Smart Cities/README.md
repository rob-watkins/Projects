
File name: 	  readme

Author: 	    Bryan Kline

Date:			    December 6, 2017

Description:	  Compilation and execution instructions and dependency information, created for networking
                course at UNR (CPE400, Fall 2017) in collaboration with Ryan Lieu and Robert Watkins.

Project Name:   Traffic Simulation for Smart Cities - Software Defined Network for Traffic Management
Purpose:        Terminal-based city map and traffic simulation tool which allows for the definition of 
                arbitrarily large city maps and car counts; models the city as a Software Defined Network 
                and demonstrates the effects of road closure and traffic congestion on map throughput, and 
                allows for dynamic smart car rerouting using Dijkstra’s algorithm.



                                                                                                        
For detailed program documentation, see CPE400 Project 6 Documentation.pdf                            
                                                                                                       


To be built and executed on Linux distributions Xubuntu or Ubuntu.

To Compile:

        In order to compile the program, a makefile is included in the software package.  To build the system  
        navigate to the directory containing the package, open terminal, and type:

            make

To Execute:

        In order to run the program, after building the program, type:

            ./SDNCity

        This will the open the program and prompt the user with the menu system where the city map can be constructed
        and the simulations can be run manually.  An alternative to this is to preload menu commands in a text file and
        then redirect input into the program.  Sample input files are provided in this package, which can be used as
        follows:

            ./SDNCity < input0 

