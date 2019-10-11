
//main.cpp
//Authors:
//			Bryan Kline
//			Ryan Lieu
//			Robert Watkins
//December 6, 2017
//CPE400 Project - #6 Software Defined Network for Traffic Management

#include <iostream>
#include <fstream>
#include <cstring>
#include "System.h"
#include "Intersection.h"

using namespace std;

#define SUCCESS 0
#define TWO_DIGIT 2
#define TEN 10
#define ASCII_OFFSET 48
#define MAX_SIZE 100

void initializeDestinations(char* destinationList[], bool initialize);
void setDestinations(char* destinationList[], char* intersectionName, int totalCars);
void integerToAlphaNumeric(int value, char* buffer, int lastIndex);

int main(int argc, char* argv[])
{
	int cars, time, error, choice = 0, delays = 1, timeDelayed, file, randomValue;
	char firstBuffer[MAX_SIZE];
	char secondBuffer[MAX_SIZE];
	char delayIntersection[MAX_SIZE];
	char fileName[] = "output.txt";
	char* destinations[MAX_SIZE];
	bool intersectionsAdded = false, roadsAdded = false, trajectoriesSet = false, delayAdded = false, toFile = true;
	ofstream outFile;
	System regularSystem;
	System delayedSystem;

	initializeDestinations(destinations, true);

	if(toFile)
	{	
		outFile.open(fileName);
	}	

	while(choice != 10)
	{	
		cout << endl << "Smart City System Map" << endl << endl;
		cout << "Load all intersections in the city, the number of cars at each intersection at time zero, " << endl;
		cout << "then load all roads connecting each intersection along with the time it takes to traverse " << endl;
		cout << "that road.  The maximum number of roads an intersection can have is four and the time to " << endl;
		cout << "traverse any road must be greater than zero.  To finish the system map, the cars which ";
		cout << "have been added then need to have their trajectories set." << endl << endl;
		cout << "0)  Output Options" << endl;
		cout << "1)  Add Intersection" << endl;
		cout << "2)  Add Road" << endl;
		cout << "3)  Set Car Trajectories" << endl;
		cout << "4)  Add Delay" << endl;
		cout << "5)  Calculate OSPF" << endl;
		cout << "6)  Display Tables for System with Delay" << endl;
		cout << "7)  Display Tables for System with Traffic Congestion" << endl;
		cout << "8)  Display Intersection, Road and Car Details" << endl;
		cout << "9)  Disply Program Details" << endl;
		cout << "10) Exit" << endl;
		cout << "Enter a menu option:" << endl;
		cin >> choice;

		switch(choice)
		{
			case 0:

				cout << "Output Options" << endl;	
				cout << "Enter 0 to output to the screen only, enter 1 to output to both the screen and a file named ";
				cout << "output.txt:" << endl;
				cin >> file;

				if(file == 0)
				{
					toFile = false;
				}	
				else
				{
					toFile = true;	
				}	

			break;

			case 1:

				if(!roadsAdded && !trajectoriesSet && !delayAdded)
				{	
					cout << "Add Intersection" << endl;
					cout << "Enter a unique name for the intersection:" << endl;
					cin >> firstBuffer;
					cout << "Enter the number of cars at the intersection:" << endl;
					cin >> cars;

					error = regularSystem.addIntersection(firstBuffer, cars);
					error = delayedSystem.addIntersection(firstBuffer, cars);
					regularSystem.errorControl(error);

					if(error != SUCCESS)
					{
						cout << "The intersection was not added to the system." << endl;
					}

					intersectionsAdded = true;
				}
				else
				{
					cout << "The map has already been defined." << endl;
				}	

			break;

			case 2:

				if(intersectionsAdded && !trajectoriesSet && !delayAdded)
				{
					cout << "Add Road" << endl;
					cout << "Enter the name of an existing intersection:" << endl;
					cin >> firstBuffer;
					cout << "Enter the name of another existing intersection:" << endl;
					cin >> secondBuffer;
					cout << "Enter the time needed to traverse the road:" << endl;
					cin >> time;

					error = regularSystem.addRoad(firstBuffer, secondBuffer, time);
					error = delayedSystem.addRoad(firstBuffer, secondBuffer, time);
					regularSystem.errorControl(error);

					if(error != SUCCESS)
					{
						cout << "The road was not added to the system." << endl;
					}

					roadsAdded = true;
				}
				else
				{
					cout << "Intersections must be added first." << endl;
				}	

			break;

			case 3:

				if(intersectionsAdded && roadsAdded && !delayAdded)
				{	
					cout << endl << "Set Car Trajectories" << endl;
					cout << "Enter the name of an existing intersection:" << endl;
					cin >> firstBuffer;
					cars = regularSystem.getTotalCars(firstBuffer);
					error = cars;

					if(error == -1)
					{
						regularSystem.errorControl(error);
						cout << "The car trajectories were not set." << endl;
					}	
					else
					{
						setDestinations(destinations, firstBuffer, cars);
						error = regularSystem.checkCarTrajectories(firstBuffer, destinations);
						
						if(error == SUCCESS)
						{
							regularSystem.setCarTrajectories(firstBuffer, destinations);
							delayedSystem.setCarTrajectories(firstBuffer, destinations);
						}
						else
						{
							regularSystem.errorControl(error);
							cout << "The car trajectories were not set." << endl;
						}	

						initializeDestinations(destinations, false);
					}

					trajectoriesSet = true;
				}
				else
				{
					cout << "Intersections and roads must be added first." << endl;
				}		

			break;

			case 4:

				if(intersectionsAdded && roadsAdded && trajectoriesSet)
				{	
					cout << "Add Delay" << endl;

					while(delays > 0)
					{	
						cout << "Enter an intersection at which the delay should occur:" << endl;
						cin >> firstBuffer;
						cout << "Enter the duration of the delay:" << endl;
						cin >> timeDelayed;

						error = delayedSystem.addDelay(firstBuffer, timeDelayed);
						strcpy(delayIntersection, firstBuffer);

						if(error != SUCCESS || timeDelayed < 0)
						{
							cout << "The delay was not set." << endl;
							delays = 1;
						}	

						delays--;
					}	

					delayAdded = true;
				}	
				else
				{
					cout << "Intersections, roads, and car trajectories must be added first." << endl;
				}	

			break;


			case 5:
				
				if(intersectionsAdded && roadsAdded && trajectoriesSet && delayAdded)
				{	
					cout << endl << "Calculate OSPF" << endl;
					regularSystem.OSPF();
					delayedSystem.OSPF();

					regularSystem.applyDelay(delayIntersection, timeDelayed);
				}	

			break;

			case 6:

				cout << endl << "Display Tables for System with Delay" << endl;
				if(toFile)
				{
					outFile << endl << "Display Tables for System with Delay" << endl;
				}


				cout << endl << "System Path Table (no delay):" << endl;
				if(toFile)
				{
					outFile << endl << "System Path Table (no delay):" << endl;
				}	
				regularSystem.outputSystemTables(toFile, outFile, 1);
				
				cout << endl << "System Path Table (delay with re-route):" << endl;
				if(toFile)
				{
					outFile << endl << "System Path Table (delay with re-route):" << endl;
				}
				delayedSystem.outputSystemTables(toFile, outFile, 0);
		
			break;

			case 7:

				cout << endl << "Display Tables for System with Traffic Congestion" << endl;
				if(toFile)
				{
					outFile << endl << "Display Tables for System with Traffic Congestion" << endl;
				}

				cout << endl << "System Path Table (no congestion):" << endl;
				if(toFile)
				{
					outFile << endl << "System Path Table (no congestion):" << endl;
				}
				regularSystem.outputSystemTables(toFile, outFile, 0);

				cout << endl << "System Path Table (congestion, no correction):" << endl;
				if(toFile)
				{
					outFile << endl << "System Path Table (congestion, no correction):" << endl;
				}
				regularSystem.simulateTraffic();
				regularSystem.outputSystemTables(toFile, outFile, 2);


			break;

			case 8:

				regularSystem.displaySystem(false, outFile);
				//delayedSystem.displaySystem(false, outFile);

			break;

			case 9:

				cout << "Program Details" << endl;
				cout << "This program was created by Bryan Kline, Ryan Lieu, and Robert Watkins for CPE400 at UNR, Fall 2017." << endl;
				cout << "The program is intended to simulate a smart city, a city street map with self driving cars which " << endl;
				cout << "are controlled by a Software Defined Network.  The user first must created all the nodes, " << endl;
				cout << "intersections, in the system, specifying how many cars start at each intersection, then connect " << endl;
				cout << "all nodes with edges, roads, specifying each road's weight or the time needed to traverse it, followed " << endl;
				cout << "by the trajectories for all cars.  The user may also specify whether there are delays at given " << endl;
				cout << "intersections and how long they last.  The system then will simulate the effect of those delays and " << endl;
				cout << "how the cars should divert around the delay using OSPF, displaying time and path tables for all cars " << endl;
				cout << "in the system with and without the delay.  The system will also simulate the effect of traffic on " << endl;
				cout << "the system at each time, the more cars that are at a given intersection at a given time the more " << endl;
				cout << "weight the roads going out of that intersection will have.  Again, the effect of traffic is shown by " << endl;
				cout << "displaying time and path table for all cars with and without traffic, as well as a novel approach " << endl;
				cout << "to handling traffic congestion by displaying an additional time and path table on the system " << endl;
				cout << "where cars with the same trajectories are considered one singular car as they can be controlled " << endl;
				cout << "by the smart city in such a way so that they do not cause congestion by in a sense acting as one " << endl;
				cout << "larger vehicle.  A summary of the system components are printed to the screen and written to " << endl;
				cout << "a file.  Similarly, all time and path tables are printed out to the screen and written to a " << endl;
				cout << "file." << endl;

			break;

			case 10:

				cout << "Exiting." << endl;

			break;

			default:

				cout << "Invalid slection, enter a valid menu option." << endl << endl;

			break;
		}

		cin.clear();
		cin.ignore();
	}	

	if(toFile)
	{	
		outFile.close();
	}	

	return SUCCESS;
}

//name:			initializeDestinations	
//description:	takes in a string array and frees allocated memory in the array if
//				it exists and sets the char pointers to NULL
//parameters:	takes in the string array to be initialized and a bool corresponding
//				to whether or not the array currently has memory allocated to its 
//				elements or not
//return:		void
void initializeDestinations(char* destinationList[], bool initialize)
{
	int index;

	for(index = 0; index < MAX_SIZE; index++)
	{
		if(!initialize && destinationList[index] != NULL)
		{
			delete destinationList[index];
		}

		destinationList[index] = NULL;
	}
}

//name:			setDestinations	
//description:	loads a string array with all the destinations for a particular intersection
//				which is then used passed to a System method to set the car destinations 
//parameters:	takes in a string array which takes the destinations, a string containing
//				the current intersection, and an int corresponding to the number of cars there
//return:		void
void setDestinations(char* destinationList[], char* intersectionName, int totalCars)
{
	int index, length;
	char buffer[MAX_SIZE];

	for(index = 0; index < totalCars; index++)
	{
		integerToAlphaNumeric(index, buffer, TWO_DIGIT);
		cout << "Enter the destination for car " << intersectionName << buffer;
		cout << " from intersection " << intersectionName << ": " << endl;
		cin >> buffer;

		length = strlen(buffer) + 1;
		destinationList[index] = new char[length];
		strcpy(destinationList[index], buffer);
	}
}

//name:			integerToAlphaNumeric	
//description:	takes in an int and converts it to a string
//parameters:	the int to be converted, the string where it is be stored, the size 
//				of the string 	
//return:		void
void integerToAlphaNumeric(int value, char* buffer, int lastIndex)
{
	int index;

	for(index = lastIndex - 1; index >= 0; index--)
	{	
		buffer[index] = (value % TEN) + ASCII_OFFSET;
		value = value / TEN;
	}	

	buffer[lastIndex] = '\0';
}	