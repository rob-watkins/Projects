
//Intersection.cpp
//Authors:
//			Bryan Kline
//			Ryan Lieu
//			Robert Watkins
//December 6, 2017
//CPE400 Project - #6 Software Defined Network for Traffic Management

#include <cstdlib>
#include <cstring>
#include "Intersection.h"
#include "System.h"

#ifndef INTERSECTION_CPP
#define INTERSECTION_CPP

#define TEN 10
#define ASCII_OFFSET 48
#define MAX_SIZE 100

void integerToAlphaNumericCoversion(int value, char* buffer, int lastIndex);

//name:			Intersection	
//description:	parameterized constructor, allocates memory for the Car array then
//				allocates memory for strings for each car, and sets the Road array to
//				defaults, the start of the Road points at this intersection, the other
//				end points to NULL
//parameters:	takes in a string corresponding to the name of the Intersection and an int
//				for the number of Car objects there
//return:		none
Intersection:: Intersection(char* intersectionName, int totalCars)
{
	int index, length = strlen(intersectionName) + 1;
	char buffer[MAX_SIZE];
	
	name = new char[length];
	strcpy(name, intersectionName);

	roadCount = 0;

	if(totalCars < MAX_CARS)
	{
		carCount = totalCars;
		cars = new Car[carCount];
	}
	else
	{
		carCount = MAX_CARS;
		cars = new Car[MAX_CARS];
	}

	for(index = 0; index < carCount; index++)
	{
		cars[index].name = new char[length + 2];
		strcpy(cars[index].name, intersectionName);
		integerToAlphaNumericCoversion(index, buffer, 2);
		strcat(cars[index].name, buffer);
		cars[index].trajectory[0] = new char[length];
		strcpy(cars[index].trajectory[0], intersectionName);
		cars[index].trajectory[1] = NULL;
		cars[index].naiveOSPFPath = NULL;
	}

	for(index = 0; index < MAX_ROADS; index++)
	{
		connectedRoads[index].time = 0;

		connectedRoads[index].intersections[THIS_INTERSECTION] = this;
		connectedRoads[index].intersections[TO_INTERSECTION] = NULL;
	}

	next = NULL;
}

//name:			~Intersection	
//description:	destructor, frees all dynamically allocated memory
//parameters:	none
//return:		none
Intersection:: ~Intersection()
{
	delete[] name;

	//TO DO: DEALLOCATE DYNAMICALLY ALLOCATED MEMORY	
}

//name:			integerToAlphaNumericCoversion	
//description:	takes in an int and converts it to a string
//parameters:	the int to be converted, the string where it is be stored, the size 
//				of the string 
//return:		void
void integerToAlphaNumericCoversion(int value, char* buffer, int lastIndex)
{
	int index;

	for(index = lastIndex - 1; index >= 0; index--)
	{
		buffer[index] = (value % TEN) + ASCII_OFFSET;
		value = value / TEN;
	}

	buffer[lastIndex] = '\0';
}

#endif
