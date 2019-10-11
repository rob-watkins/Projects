
//System.cpp
//Authors:
//			Bryan Kline
//			Ryan Lieu
//			Robert Watkins
//December 6, 2017
//CPE400 Project - #6 Software Defined Network for Traffic Management

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstring>
#include "System.h"
#include "Intersection.h"

#ifndef SYSTEM_CPP
#define SYSTEM_CPP

using namespace std;

#define THIS_INTERSECTION 0
#define TO_INTERSECTION 1
#define THRESHOLD 2
#define TEN 10
#define ASCII_OFFSET 48
#define MAX_SIZE 100

//name:			System		
//description:	default constructor, sets data members to default values 
//parameters:	none
//return:		none
System:: System()
{
	intersectionCount = 0;
	totalCars = 0;
	maxPathLength = 0;
	dijkstraRun = 0;
	latestTime = 0;
	threshold = THRESHOLD;

	lowestTime = INFINITY;
	lowestDelayTime = INFINITY;
	lowestCongestionTime = INFINITY;
	highestTime = 0;
	highestDelayTime = 0;
	highestCongestionTime = 0;
	averageTravelTime = 0.0;
	averageTravelTimeDelay = 0.0;
	averageTravelTimeCongestion = 0.0;

	intersectionList = NULL;
}

//name:			~System	
//description:	destructor, frees all dynamically allocated memory
//parameters:	none
//return:		none
System:: ~System()
{
	Intersection* firstTemp = intersectionList;
	Intersection* secondTemp = intersectionList;

	while(firstTemp != NULL)
	{
		firstTemp = firstTemp->next;
		delete secondTemp;
		secondTemp = firstTemp;
	}	

	intersectionList = NULL;
}

//name:			isEmpty	
//description:	checks whether or not the Intersection list exists or not
//parameters:	none
//return:		returns a bool corresponding to whether or not the list exists
bool System:: isEmpty()
{
	int index, indexInner;

	if(intersectionList == NULL)
	{
		return true;
	}	

	return false;

	//TO DO: DEALLOCATE DYNAMICALLY ALLOCATED MEMORY
	/*
	for(index = 0; index < totalCars; index++)
	{
		for(indexInner = 0; indexInner < maxPathLength; indexInner++)
		{
			delete[] systemTimeTable[index][indexInner].name;
			delete[] systemTimeTableDelay[index][indexInner].name;
			delete[] systemTimeTableCongestion[index][indexInner].name;
		}

		delete[] systemTimeTable[index];
		delete[] systemTimeTableDelay[index];
		delete[] systemTimeTableCongestion[index];
	}

	delete systemTimeTable;
	delete systemTimeTableDelay;
	delete systemTimeTableCongestion;	
	*/	
}

//name:			addIntersection	
//description:	creates an Intersection object, names it the string passed in and sets
//				the number of cars to the int passed in, and connects the newly created 
//				Intersection to the Intersection list
//parameters:	takes in a string corresponding to the name of the new Intersection and
//				an int for the number of Car objects at that Intersection
//return:		returns an int which is an error code which reports what error was encountered
int System:: addIntersection(char* newIntersection, int cars)
{
	Intersection* temp = getIntersection(newIntersection);

	if(temp != NULL)
	{
		return INTERSECTION_EXISTS;
	}	

	if(isEmpty())
	{
		intersectionList = new Intersection(newIntersection, cars);

		if(intersectionList == NULL)
		{
			return MEMORY_FAULT;
		}
	}
	else
	{	
		temp = intersectionList;

		while(temp->next != NULL)
		{
			temp = temp->next;
		}	
	
		temp->next = new Intersection(newIntersection, cars);

		if(temp->next == NULL)
		{
			return MEMORY_FAULT;
		}	
	}	

	intersectionCount++;
	return SUCCESES;
}

//name:			addRoad	
//description:	takes in to and from Intersection names and connects the Road array at the from 
//				Intersection to the to Intersection, and sets the time it takes to traverse that
//				Road object
//parameters:	takes in two strings, the to and the from Intersection names, and the time it
//				takes to traverse that Road
//return:		returns an int which is an error code which reports what error was encountered
int System:: addRoad(char* toIntersection, char* fromIntersection, int time)
{
	int toIndex = 0, fromIndex = 0;
	Intersection* toTemp = getIntersection(toIntersection);
	Intersection* fromTemp = getIntersection(fromIntersection);

	if(toTemp == NULL || fromTemp == NULL)
	{
		return NOT_FOUND;
	}	

	if(toTemp->roadCount >= MAX_ROADS || fromTemp->roadCount >= MAX_ROADS)
	{
		return MAX_ROADS_EXCEEDED;
	}	

	if(time <= 0)
	{
		return ZERO_TIME;
	}

	while(toTemp->connectedRoads[toIndex].time != 0)
	{
		if(toTemp->connectedRoads[toIndex].intersections[TO_INTERSECTION] == fromTemp)
		{
			return ROAD_EXISTS;
		}

		toIndex++;
	}	
	toTemp->connectedRoads[toIndex].time = time;
	toTemp->connectedRoads[toIndex].intersections[TO_INTERSECTION] = fromTemp;
	toTemp->roadCount++;

	while(fromTemp->connectedRoads[fromIndex].time != 0)
	{
		if(fromTemp->connectedRoads[fromIndex].intersections[TO_INTERSECTION] == toTemp)
		{
			return ROAD_EXISTS;
		}

		fromIndex++;
	}	
	fromTemp->connectedRoads[fromIndex].time = time;
	fromTemp->connectedRoads[fromIndex].intersections[TO_INTERSECTION] = toTemp;
	fromTemp->roadCount++;

	return SUCCESES;
}

//name:			setCarTrajectories		
//description:	sets the destinations of the Cars in the Car array at an Intersection to all the
//				destinations entered as the string array parameter 
//parameters:	takes in a string which is the name of the current Intersection as well as a
//				string array which holds the names of the other Intersections to which the
//				Cars at that Intersection are traveling
//return:		returns an int which is an error code which reports what error was encountered
int System:: setCarTrajectories(char* fromIntersection, char* destinationList[])
{
	int index, length;	
	Intersection* fromTemp = getIntersection(fromIntersection);

	if(fromTemp == NULL)
	{
		return NOT_FOUND;
	}

	for(index = 0; index < fromTemp->carCount; index++)
	{
		length = strlen(destinationList[index]) + 1;
		fromTemp->cars[index].trajectory[1] = new char[length];
		strcpy(fromTemp->cars[index].trajectory[1], destinationList[index]);
	}

	return SUCCESES;
}

//name:			checkCarTrajectories	
//description:	iterates through the string array passed in as a parameter to test whether or not
//				all destinations in the array are valid from the current Intersection
//parameters:	takes in a string which is the name of the current Intersection as well as a
//				string array which holds the names of the other Intersections to which the
//				Cars at that Intersection are traveling
//return:		returns an int which is an error code which reports what error was encountered
int System:: checkCarTrajectories(char* fromIntersection, char* destinationList[])
{
	int index = 0;
	Intersection* temp = getIntersection(fromIntersection);

	if(temp == NULL)
	{
		return NONEXISTENT_INTERSECTION;
	}

	while(destinationList[index] != NULL)
	{
		temp = getIntersection(destinationList[index]);

		if(temp == NULL)
		{
			return NONEXISTENT_DESTINATION;
		}

		if(strcmp(fromIntersection, destinationList[index]) == 0)
		{
			return NONEXISTENT_DESTINATION;
		}

		index++;
	}

	return SUCCESES;
}

//name:			addDelay	
//description:	takes in the name of an Intersection, moves to that Intersection, and iterates through
//				the Roads at that Intersection and adds a delay to each one
//parameters:	takes in a string corresponding to the Intersection at which the delay occurs and an int 
//				which is the amount of time the delay occurs for
//return:		returns an error code as an int which reports whether or not the addition of
//				the delay was successful
int System:: addDelay(char* fromIntersection, int delay)
{
	int index;
	Intersection* fromTemp = getIntersection(fromIntersection);

	if(fromTemp == NULL)
	{
		return NONEXISTENT_INTERSECTION;
	}

	for(index = 0; index < fromTemp->roadCount; index++)
	{
		fromTemp->connectedRoads[index].time += delay;
	}	

	return SUCCESES;
}

//name:			getTotalCars	
//description:	returns the number of Car objects at a given Intersection
//parameters:	takes in a string corresponding to the name of the Intersection from which to 
//				get the Cars
//return:		returns and int which is either an error code which reports what error was 
//				or the number of Cars at that Intersection encountered
int System:: getTotalCars(char* intersectionName)
{
	Intersection* fromTemp = getIntersection(intersectionName);

	if(fromTemp == NULL)
	{
		return NONEXISTENT_INTERSECTION;
	}

	return fromTemp->carCount;
}

//name:			getSystemCars	
//description:	returns the total number of Cars in the system
//parameters:	none
//return:		returns an int corresponding to the total number of Car objects in the 
//				entire system
int System:: getSystemCars()
{
	int totalCars = 0;
	Intersection* fromTemp = intersectionList;

	while(fromTemp != NULL)
	{
		totalCars += fromTemp->carCount;
		fromTemp = fromTemp->next;
	}	

	return totalCars;
}

//name:			getIntersection	
//description:	takes in the name of an Intersection and returns a reference to it if it 
//				exists, otherwise NULL is returned
//parameters:	takes in a string which is the name of the Intersection for which a reference is
//				requested
//return:		returns a pointer to an Intersection which is the Intersection requested if it
//				exists, returns NULL if it doesn't exist
Intersection* System:: getIntersection(char* intersectionName)
{
	Intersection* temp = intersectionList;

	while(temp != NULL)
	{
		if(strcmp(temp->name, intersectionName) == 0)
		{
			return temp;
		}	

		temp = temp->next;
	}	

	return temp;
}

//name:			OSPF		
//description:	all Intersections in the system are iterated through, the shortest paths to every other
//				intersection are calculated using Dijkstra's algorithm and stored in a table after
//				initializing the table, then the shortest paths for all Cars in the system are calculated
//				using each Intersection's OSPF table, and then the system time tables are populated with
//				all Cars' shortest paths 
//parameters:	none
//return:		void
void System:: OSPF()
{
	Intersection* temp = intersectionList;

	while(temp != NULL)
	{
		initializeOSPFTable(temp->name);
		runDijkstra(temp->name);

		temp = temp->next;
	}

	calculatePaths();
	initializeSystemTable();
}

//name:			initializeOSPFTable	
//description:	dynamically allocates memory for the OSPF table at a given Intersection,
//				sets each DijkstraNode object in the table to default values, then the 
//				Intersection list is moved through and the name of each Intersection
//				is added to the first row of the table which acts as column labels
//parameters:	takes in a string corresponding to the name of the Intersection whose
//				OSPF table is to be initialized
//return:		void
void System:: initializeOSPFTable(char* intersectionName)
{
	int index, innerIndex, length;
	Intersection* intersection = getIntersection(intersectionName);
	Intersection* temp;

	intersection->OSPFTable = new DijkstraNode*[intersectionCount];
	for(index = 0; index < intersectionCount; index++)
	{
		intersection->OSPFTable[index] = new DijkstraNode[intersectionCount - 1];
	}

	for(index = 0; index < intersectionCount; index++)
	{
		for(innerIndex = 0; innerIndex < intersectionCount - 1; innerIndex++)
		{
			intersection->OSPFTable[index][innerIndex].through = NULL;
			intersection->OSPFTable[index][innerIndex].cost = INFINITY;
			intersection->OSPFTable[index][innerIndex].done = false;	
		}
	}		

	temp = intersectionList;
	index = 0;
	while(temp != NULL && index < intersectionCount - 1)
	{
		if(strcmp(temp->name, intersectionName) != 0)
		{
			length = strlen(temp->name) + 1;
			intersection->OSPFTable[0][index].through = new char[length];
			strcpy(intersection->OSPFTable[0][index].through, temp->name);
			index++;
		}	

		temp = temp->next;
	}	

	dijkstraRun = 1;
}

//name:			runDijkstra	
//description:	runs Dijkstra's algorithm at a given Intersection, calculating the shortest
//				paths from that Intersection to all other Intersections and populating the
//				OSPF table with all the costs to each Intersection
//parameters:	takes in a string which corresponds to the Intersection at which Dijkstra's
//				algorithm shall be run
//return:		void
void System:: runDijkstra(char* intersectionName)
{
	int index, innerIndex, min, minIndex, length, roads, roadIndex, time, totalTime, tableIndex;
	char* toName;
	Intersection* home = getIntersection(intersectionName);	
	Intersection* current = home; 	

	if(dijkstraRun == 1)
	{	
		for(index = 1; index < intersectionCount; index++)
		{
			for(innerIndex = 0; innerIndex < intersectionCount - 1; innerIndex++)
			{
				home->OSPFTable[index][innerIndex].cost = home->OSPFTable[index - 1][innerIndex].cost;
				length = strlen(home->OSPFTable[index - 1][innerIndex].through) + 1;
				home->OSPFTable[index][innerIndex].through = new char[length];
				strcpy(home->OSPFTable[index][innerIndex].through, home->OSPFTable[index - 1][innerIndex].through);		
				
				time = getTime(current->name, home->OSPFTable[0][innerIndex].through);
				
				if(!home->OSPFTable[index][innerIndex].done && time  > 0)
				{

					if(current == home)
					{
						totalTime = 0; 
					}
					else
					{
						tableIndex = getIndex(home->name, current->name);
						totalTime = home->OSPFTable[index - 1][tableIndex].cost;
					}

					totalTime += time; 

					if(home->OSPFTable[index][innerIndex].cost > totalTime)
					{
						home->OSPFTable[index][innerIndex].cost = totalTime;
						length = strlen(current->name) + 1;
						if(home->OSPFTable[index][innerIndex].through != NULL)
						{
							delete home->OSPFTable[index][innerIndex].through;
						}	
						home->OSPFTable[index][innerIndex].through = new char[length];
						strcpy(home->OSPFTable[index][innerIndex].through, current->name);					
					}	
				}	
			}	

			min = INFINITY;
			minIndex = -1;
			for(innerIndex = 0; innerIndex < intersectionCount - 1; innerIndex++)
			{
				if(!home->OSPFTable[index][innerIndex].done && home->OSPFTable[index][innerIndex].cost < min)
				{
					min = home->OSPFTable[index][innerIndex].cost;
					minIndex = innerIndex;
				}
			}	

			for(innerIndex = 0; innerIndex < intersectionCount; innerIndex++)
			{
				home->OSPFTable[innerIndex][minIndex].done = true;
			}

			current = getIntersection(home->OSPFTable[0][minIndex].through);
		}		

		dijkstraRun = 2;
	}	
}

//name:			calculatePaths	
//description:	iterates through the Intersection list and creates Path lists for the
//				Cars at that intersection and then consults the OSPF table at that
//				Intersection to create the shortest path for that Car and the Path
//				list is filled in with the intermediate Intersections and costs for 
//				each move for that Car through the system
//parameters:	none
//return:		void
void System:: calculatePaths()
{
	int index, indexInner, destinationIndex, length;
	Intersection* temp = intersectionList;
	struct Path* tempPath;

	if(dijkstraRun == 2)
	{	
		for(index = 0; temp != NULL && index < intersectionCount; index++)
		{
			for(indexInner = 0; indexInner < temp->carCount; indexInner++)
			{
				temp->cars[indexInner].naiveOSPFPath = new Path[1];
				length = strlen(temp->cars[indexInner].trajectory[1]) + 1;
				temp->cars[indexInner].naiveOSPFPath->name = new char[length];
				strcpy(temp->cars[indexInner].naiveOSPFPath->name, temp->cars[indexInner].trajectory[1]);
				destinationIndex = getIndex(temp->name, temp->cars[indexInner].trajectory[1]);
				temp->cars[indexInner].naiveOSPFPath->cost = temp->OSPFTable[intersectionCount - 1][destinationIndex].cost;

				tempPath = temp->cars[indexInner].naiveOSPFPath;

				while(strcmp(temp->name, tempPath->name) != 0)
				{
					tempPath->next = new Path[1];
					tempPath = tempPath->next;

					length = strlen(temp->OSPFTable[intersectionCount - 1][destinationIndex].through) + 1;
					tempPath->name = new char[length];
					strcpy(tempPath->name, temp->OSPFTable[intersectionCount - 1][destinationIndex].through);	

					destinationIndex = getIndex(temp->name, temp->OSPFTable[intersectionCount - 1][destinationIndex].through);
					if(destinationIndex < 0)
					{
						tempPath->cost = 0;
					}	
					else	
					{
						tempPath->cost = temp->OSPFTable[intersectionCount - 1][destinationIndex].cost;
					}	
				}
				tempPath->next = NULL;	
			}

			temp = temp->next;
		}	

		dijkstraRun = 3;
	}	
}

//name:			getIndex	
//description:	checks the OSPF table at a given Intersection to see if a requested
//				Intersection is in the table and if so its index in the table is returned
//parameters:	takes in a string which is the current Intersection and another 
//				string which is the Intersection whose index should be returned
//return:		returns an int corresponding to the index in the OSPF table at a given
//				Intersection another Intersection resides at, -1 is returned if either 
//				the current Intersection doesn't exists or the requested table Intersection
//				doesn't exist 
int System:: getIndex(char* intersectionName, char* label)
{
	int index;
	Intersection* temp = getIntersection(intersectionName);

	if(temp == NULL)
	{
		return -1;
	}	

	for(index = 0; index < intersectionCount - 1; index++)
	{
		if(strcmp(temp->OSPFTable[0][index].through, label) == 0)
		{
			return index;
		}	
	}	

	return -1;
}

//name:			getTime	
//description:	takes in the names of two Intersections and returns the time needed to
//				traverse the Road between them if it exists, zero is returned if it doesn't
//parameters:	takes in two strings corresponding to the start and end Intersections
//return:		an int corresponding to the time needed to traverse a given Road,
//				if the Road doesn't exist then zero is returned  
int System:: getTime(char* fromIntersection, char* toIntersection)
{
	int index, time = 0;
	Intersection* temp = getIntersection(fromIntersection);

	for(index = 0; index < temp->roadCount; index++)
	{
		if(strcmp(temp->connectedRoads[index].intersections[1]->name, toIntersection) == 0)
		{
			time = temp->connectedRoads[index].time;
		}	
	}

	return time;
}

//name:			initializeSystemTable	
//description:	three copies of a table of Car trajectories are created of size Cars in the system,
//				rows, by the longest path, columns, by iterating through all Cars in the system
//				adding their shortest paths to each table
//parameters:	none
//return:		void
void System:: initializeSystemTable()
{
	int index, innerIndex, count, length, tableRow = 0;
	struct Path* path;
	Intersection* intersection;

	if(dijkstraRun == 3)
	{	
		intersection = intersectionList;
		totalCars = getSystemCars();

		while(intersection != NULL)
		{
			for(index = 0; index < intersection->carCount; index++)
			{
				count = 0;
				path = intersection->cars[index].naiveOSPFPath;

				if(index == 0 && intersection->cars[index].naiveOSPFPath->cost > latestTime)
				{
					latestTime = intersection->cars[index].naiveOSPFPath->cost;
				}

				while(path != NULL)
				{
					count++;
					path = path->next;
				}	

				if(count > maxPathLength)
				{
					maxPathLength = count;
				}	
			}	

			intersection = intersection->next;
		}	

		systemTimeTable = new Path*[totalCars];
		systemTimeTableDelay = new Path*[totalCars];
		systemTimeTableCongestion = new Path*[totalCars];

		for(index = 0; index < totalCars; index++)
		{
			systemTimeTable[index] = new Path[maxPathLength];
			systemTimeTableDelay[index] = new Path[maxPathLength];
			systemTimeTableCongestion[index] = new Path[maxPathLength];

			for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
			{
				systemTimeTable[index][innerIndex].cost = -1;
				systemTimeTable[index][innerIndex].combine = false;
				systemTimeTable[index][innerIndex].name = NULL;
				systemTimeTable[index][innerIndex].next = NULL;

				systemTimeTableDelay[index][innerIndex].cost = -1;
				systemTimeTableDelay[index][innerIndex].combine = false;
				systemTimeTableDelay[index][innerIndex].name = NULL;
				systemTimeTableDelay[index][innerIndex].next = NULL;

				systemTimeTableCongestion[index][innerIndex].cost = -1;
				systemTimeTableCongestion[index][innerIndex].combine = false;
				systemTimeTableCongestion[index][innerIndex].name = NULL;
				systemTimeTableCongestion[index][innerIndex].next = NULL;
			}	
		}	

		intersection = intersectionList;

		while(intersection != NULL)
		{
			for(index = 0; index < intersection->carCount; index++)
			{
				count = 0;
				path = intersection->cars[index].naiveOSPFPath;

				while(path != NULL)
				{
					count++;
					path = path->next;
				}	

				path = intersection->cars[index].naiveOSPFPath;

				for(innerIndex = count - 1; innerIndex >= 0 ; innerIndex--)
				{
					systemTimeTable[tableRow][innerIndex].cost = path->cost;
					length = strlen(path->name) + 1;
					systemTimeTable[tableRow][innerIndex].name = new char[length];
					strcpy(systemTimeTable[tableRow][innerIndex].name, path->name);

					systemTimeTableDelay[tableRow][innerIndex].cost = path->cost;
					systemTimeTableDelay[tableRow][innerIndex].name = new char[length];
					strcpy(systemTimeTableDelay[tableRow][innerIndex].name, path->name);

					systemTimeTableCongestion[tableRow][innerIndex].cost = path->cost;
					systemTimeTableCongestion[tableRow][innerIndex].name = new char[length];
					strcpy(systemTimeTableCongestion[tableRow][innerIndex].name, path->name);

					path = path->next;
				}	

				tableRow++;
			}	

			intersection = intersection->next;
		}

		dijkstraRun = 4;
	}			
}

//name:			applyDelay	
//description:	one of the system tables with the default shortest Car paths loaded into it is iterated through
//				and changed in such a way so that delays are added to the times in the table
//parameters:	takes in a string corresponding to the Intersection at which the delay occurs and an int 
//				which is the amount of time the delay occurs for 
//return:		void
void System:: applyDelay(char* fromIntersection, int delay)
{
	int index, outerIndex, innerIndex;

	for(outerIndex = 0; outerIndex < maxPathLength; outerIndex++)
	{
		for(innerIndex = 0; innerIndex < totalCars; innerIndex++)
		{
			if(systemTimeTableDelay[innerIndex][outerIndex].name != NULL &&
				strcmp(systemTimeTableDelay[innerIndex][outerIndex].name, fromIntersection) == 0)
			{
				index = outerIndex;
				while(index < maxPathLength && systemTimeTableDelay[innerIndex][index].cost != -1)
				{
					if(index != 0)
					{	
						systemTimeTableDelay[innerIndex][index].cost += delay;
					}	
					index++;
				}
			}
		}			
	}	
}

//name:			simulateTraffic	
//description:	the primary system time table containing all Cars in the system and their shortest
//				paths, and the amount of traffic at each Intersection, at each time, for each Car,
//				is compared with a traffic threshold and that then ripples forward in the table increasing
//				each Car's path times; the same process also takes place on another table which has had Cars with 
//				similar trajectories combined in order to correct for traffic congestion
//parameters:	none
//return:		void
void System:: simulateTraffic()
{
	int index, columnIndex, rowIndex, timeIndex, intersectionIndex;
	int carsAtIntersectionCongestion, carsAtIntersectionCombine, latestTimeCongestion = latestTime, latestTimeCombine = latestTime;
	char* intersectionNames[intersectionCount];
	Intersection* intersection;

	combineCars();

	if(dijkstraRun == 4)
	{
		intersection = intersectionList;
		index = 0;

		while(intersection != NULL)
		{
			intersectionNames[index] = intersection->name;
			intersection = intersection->next;
			index++;
		}	

		for(columnIndex = 0; columnIndex < maxPathLength; columnIndex++)
		{	
			for(intersectionIndex = 0; intersectionIndex < intersectionCount; intersectionIndex++)
			{
				for(timeIndex = 0; timeIndex < latestTime + 1; timeIndex++)
				{
					carsAtIntersectionCongestion = 0;
					carsAtIntersectionCombine = 0;
					for(rowIndex = 0; rowIndex < totalCars; rowIndex++)
					{
						if(systemTimeTable[rowIndex][columnIndex].cost > latestTime)
						{
							latestTime = systemTimeTable[rowIndex][columnIndex].cost;
						}	
						if(systemTimeTableCongestion[rowIndex][columnIndex].cost > latestTime)
						{
							latestTime = systemTimeTableCongestion[rowIndex][columnIndex].cost;
						}

						if(systemTimeTable[rowIndex][columnIndex].name != NULL && !systemTimeTable[rowIndex][columnIndex].combine &&
							strcmp(systemTimeTable[rowIndex][columnIndex].name, intersectionNames[intersectionIndex]) == 0 && 
							systemTimeTable[rowIndex][columnIndex].cost == timeIndex)
						{
							carsAtIntersectionCongestion++;
						}

						if(systemTimeTableCongestion[rowIndex][columnIndex].name != NULL && 
							!systemTimeTableCongestion[rowIndex][columnIndex].combine &&
							strcmp(systemTimeTableCongestion[rowIndex][columnIndex].name, intersectionNames[intersectionIndex]) == 0 && 
							systemTimeTableCongestion[rowIndex][columnIndex].cost == timeIndex)
						{
							carsAtIntersectionCombine++;
						}		
					}	

					if(carsAtIntersectionCongestion > threshold)
					{	
						for(rowIndex = 0; rowIndex < totalCars; rowIndex++)
						{
							if(systemTimeTable[rowIndex][columnIndex].name != NULL &&
								strcmp(systemTimeTable[rowIndex][columnIndex].name, intersectionNames[intersectionIndex]) == 0 &&
								systemTimeTable[rowIndex][columnIndex].cost == timeIndex)
							{	
								index = columnIndex;
								while(index < maxPathLength - 1 && systemTimeTable[rowIndex][index + 1].cost != -1)
								{
									systemTimeTable[rowIndex][index + 1].cost += carsAtIntersectionCongestion;
									index++;
								}	
							}	
						}
					}	

					if(carsAtIntersectionCombine > threshold)
					{	
						for(rowIndex = 0; rowIndex < totalCars; rowIndex++)
						{
							if(systemTimeTableCongestion[rowIndex][columnIndex].name != NULL &&
								strcmp(systemTimeTableCongestion[rowIndex][columnIndex].name, intersectionNames[intersectionIndex]) == 0 &&
								systemTimeTableCongestion[rowIndex][columnIndex].cost == timeIndex)
							{	
								index = columnIndex;
								while(index < maxPathLength - 1 && systemTimeTableCongestion[rowIndex][index + 1].cost != -1)
								{
									systemTimeTableCongestion[rowIndex][index + 1].cost += carsAtIntersectionCombine;
									index++;
								}	
							}	
						}
					}
				}	
			}	
		}
	}	
}

//name:			combineCars		
//description:	the system table which will hold the Car travel times after congestion correction has taken place
//				is iterated through and if a Car is at an Intersection at a given time and shares a portion of their
//				path with another Car at that same Intersection at that exact time, then they are combined into one
//				in order lessen the effects of traffic on congestion on the system
//parameters:	none
//return:		void
void System:: combineCars()
{
	int index, columnIndex, rowIndex, timeIndex, intersectionIndex;
	char* intersectionNames[intersectionCount];
	Intersection* intersection;

	if(dijkstraRun == 4)
	{
		intersection = intersectionList;
		index = 0;

		while(intersection != NULL)
		{
			intersectionNames[index] = intersection->name;
			intersection = intersection->next;
			index++;
		}	

		for(columnIndex = 0; columnIndex < maxPathLength; columnIndex++)
		{	
			for(intersectionIndex = 0; intersectionIndex < intersectionCount; intersectionIndex++)
			{
				for(timeIndex = 0; timeIndex < latestTime + 1; timeIndex++)
				{
					for(rowIndex = 0; rowIndex < totalCars - 1; rowIndex++)
					{
						if(systemTimeTableCongestion[rowIndex][columnIndex].cost > latestTime)
						{
							latestTime = systemTimeTableCongestion[rowIndex][columnIndex].cost;
						}

						if(systemTimeTableCongestion[rowIndex][columnIndex].name != NULL && columnIndex < maxPathLength - 1 &&
							!systemTimeTableCongestion[rowIndex][columnIndex].combine &&
							strcmp(systemTimeTableCongestion[rowIndex][columnIndex].name, intersectionNames[intersectionIndex]) == 0 && 
							systemTimeTableCongestion[rowIndex][columnIndex].cost == timeIndex)
						{
							
							for(index = rowIndex + 1; index < totalCars; index++)
							{
								if(systemTimeTableCongestion[rowIndex][columnIndex + 1].name != NULL &&
									systemTimeTableCongestion[index][columnIndex].name != NULL && systemTimeTableCongestion[index][columnIndex + 1].name != NULL &&
									strcmp(systemTimeTableCongestion[rowIndex][columnIndex].name, systemTimeTableCongestion[index][columnIndex].name ) == 0 && 
									strcmp(systemTimeTableCongestion[rowIndex][columnIndex + 1].name, systemTimeTableCongestion[index][columnIndex + 1].name ) == 0 && 
									systemTimeTableCongestion[rowIndex][columnIndex].cost == systemTimeTableCongestion[index][columnIndex].cost &&
									systemTimeTableCongestion[rowIndex][columnIndex + 1].cost == systemTimeTableCongestion[index][columnIndex + 1].cost)
								{
									systemTimeTableCongestion[index][columnIndex].combine = true;
								}	
							}	
							
						}		
					}		
				}	
			}	
		}
	}
}

//name:			timeSummary
//description:	the three system tables which hold the simple shortest paths, the paths with delay
//				times added, and the paths with traffic congestion, are iterated through and the lowest,
//				highest, and average travel times, are calculated and stored in class data members
//parameters:	none
//return:		void
void System:: timeSummary()
{	
	int index, innerIndex;

	lowestTime = INFINITY;
	lowestDelayTime = INFINITY;
	lowestCongestionTime = INFINITY;
	highestTime = 0;
	highestDelayTime = 0;
	highestCongestionTime = 0;
	averageTravelTime = 0.0;
	averageTravelTimeDelay = 0.0;
	averageTravelTimeCongestion = 0.0;

	for(index = 0; index < totalCars; index++)
	{
		innerIndex = 0;
		while(innerIndex < (maxPathLength - 1) && systemTimeTable[index][innerIndex + 1].cost != -1)
		{
			innerIndex++;
		}	

		if(systemTimeTable[index][innerIndex].cost < lowestTime)
		{
			lowestTime = systemTimeTable[index][innerIndex].cost;
		}	
		if(systemTimeTableDelay[index][innerIndex].cost < lowestDelayTime)
		{
			lowestDelayTime = systemTimeTableDelay[index][innerIndex].cost;
		}	
		if(systemTimeTableCongestion[index][innerIndex].cost < lowestCongestionTime)
		{
			lowestCongestionTime = systemTimeTableCongestion[index][innerIndex].cost;
		}

		if(systemTimeTable[index][innerIndex].cost > highestTime)
		{
			highestTime = systemTimeTable[index][innerIndex].cost;
		}	
		if(systemTimeTableDelay[index][innerIndex].cost > highestDelayTime)
		{
			highestDelayTime = systemTimeTableDelay[index][innerIndex].cost;
		}	
		if(systemTimeTableCongestion[index][innerIndex].cost > highestCongestionTime)
		{
			highestCongestionTime = systemTimeTableCongestion[index][innerIndex].cost;
		}

		averageTravelTime += systemTimeTable[index][innerIndex].cost; 
		averageTravelTimeDelay += systemTimeTableDelay[index][innerIndex].cost; 
		averageTravelTimeCongestion += systemTimeTableCongestion[index][innerIndex].cost; 
	}			

	averageTravelTime /= totalCars;
	averageTravelTimeDelay /= totalCars;
	averageTravelTimeCongestion /= totalCars;
}

//name:			displaySystem		
//description:	moves through the Intersection list and prints to the screen, and written to a file, all 
//				the data, including the OSPF table, at each Intersection
//parameters:	takes in a bool corresponding to whether or not the Intersection data should be written
//				to a file and an ofstream object which is the stream connected to the file
//return:		void
void System:: displaySystem(bool toFile, ofstream &outFile)
{
	int index, innerIndex, dashIndex, intersectionIndex, carIndex;
	Intersection* temp = intersectionList;
	struct Path* tempPath;

	if(isEmpty())
	{
		errorControl(SYSTEM_EMPTY);
	}	
	else
	{
		cout << endl << "Display System" << endl;
		cout << "Total Intersections: " << intersectionCount << endl;
		cout << "Total Cars: " << totalCars << endl;
		while(temp != NULL)
		{
			cout << endl << "Intersection Name: " << temp->name << endl;
			cout << "Roads: " << temp->roadCount << endl;
			for(intersectionIndex = 0; intersectionIndex < temp->roadCount; intersectionIndex++)
			{
				cout << "\t(" << temp->connectedRoads[intersectionIndex].time << ", ";
				cout << "[" << (temp->connectedRoads[intersectionIndex].intersections[THIS_INTERSECTION])->name << ", ";
				cout << (temp->connectedRoads[intersectionIndex].intersections[TO_INTERSECTION])->name; 

				cout << "])" << endl;

			}

			cout << "Cars: " << temp->carCount << endl;

			for(carIndex = 0; carIndex < temp->carCount; carIndex++)
			{
				cout << "\t(Name: " << temp->cars[carIndex].name << " From: ";
				cout << temp->cars[carIndex].trajectory[0] << " To: ";
				if(temp->cars[carIndex].trajectory[1] != NULL)
				{
					cout << temp->cars[carIndex].trajectory[1]; 
				}
				else
				{
					cout << "NULL";
				}	
				cout << ")\tShortest Path:  [";

				tempPath = temp->cars[carIndex].naiveOSPFPath;
				while(tempPath != NULL)
				{
					cout << tempPath->name << ":" << tempPath->cost;
					if(tempPath->next != NULL)
					{
						cout << ", "; 
					}
					tempPath = tempPath->next;
				}	
				cout << "]" << endl;
			}

			cout << endl << "OSPF Table: (From: " << temp->name <<")" << endl;
			for(index = 0; index < intersectionCount; index++)
			{
				for(innerIndex = 0; innerIndex < intersectionCount - 1; innerIndex++)
				{
					if(temp->OSPFTable[index][innerIndex].cost != INFINITY)
					{	
						cout << temp->OSPFTable[index][innerIndex].cost << "\t";
					}
					else
					{
						if(index != 0)
						{	
							cout << "INF\t";
						}
						else
						{
							cout << "To:\t";
						}	
					}		

					if(temp->OSPFTable[index][innerIndex].through != NULL)
					{	
						cout << temp->OSPFTable[index][innerIndex].through << "\t\t";
					}
					else
					{
						cout << "-\t\t";
					}	
				}	

				if(index == 0)
				{
					cout << endl;
					for(dashIndex = 0; dashIndex < (intersectionCount - 1) * 23; dashIndex++)
					{
						cout << "-";
					}	
				}	
				cout << endl;
			}

			temp = temp->next;
		}	

		if(toFile)
		{
			temp = intersectionList;

			outFile << endl << "Display System" << endl;
			outFile << "Total Intersections: " << intersectionCount << endl;
			outFile << "Total Cars: " << totalCars << endl;
			while(temp != NULL)
			{
				outFile << endl << "Intersection Name: " << temp->name << endl;
				outFile << "Roads: " << temp->roadCount << endl;
				for(intersectionIndex = 0; intersectionIndex < temp->roadCount; intersectionIndex++)
				{
					outFile << "\t(" << temp->connectedRoads[intersectionIndex].time << ", ";
					outFile << "[" << (temp->connectedRoads[intersectionIndex].intersections[THIS_INTERSECTION])->name << ", ";
					outFile << (temp->connectedRoads[intersectionIndex].intersections[TO_INTERSECTION])->name; 

					outFile << "])" << endl;

				}

				outFile << "Cars: " << temp->carCount << endl;

				for(carIndex = 0; carIndex < temp->carCount; carIndex++)
				{
					outFile << "\t(Name: " << temp->cars[carIndex].name << " From: ";
					outFile << temp->cars[carIndex].trajectory[0] << " To: ";
					if(temp->cars[carIndex].trajectory[1] != NULL)
					{
						outFile << temp->cars[carIndex].trajectory[1]; 
					}
					else
					{
						outFile << "NULL";
					}	
					outFile << ")\tShortest Path:  [";

					tempPath = temp->cars[carIndex].naiveOSPFPath;
					while(tempPath != NULL)
					{
						outFile << tempPath->name << ":" << tempPath->cost;
						if(tempPath->next != NULL)
						{
							outFile << ", "; 
						}
						tempPath = tempPath->next;
					}	
					outFile << "]" << endl;
				}

				outFile << endl << "OSPF Table: (From: " << temp->name <<")" << endl;
				for(index = 0; index < intersectionCount; index++)
				{
					for(innerIndex = 0; innerIndex < intersectionCount - 1; innerIndex++)
					{
						if(temp->OSPFTable[index][innerIndex].cost != INFINITY)
						{	
							outFile << temp->OSPFTable[index][innerIndex].cost << "\t";
						}
						else
						{
							if(index != 0)
							{	
								outFile << "INF\t";
							}
							else
							{
								outFile << "To:\t";
							}	
						}		

						if(temp->OSPFTable[index][innerIndex].through != NULL)
						{	
							outFile << temp->OSPFTable[index][innerIndex].through << "\t\t";
						}
						else
						{
							outFile << "-\t\t";
						}	
					}	

					if(index == 0)
					{
						outFile << endl;
						for(dashIndex = 0; dashIndex < (intersectionCount - 1) * 23; dashIndex++)
						{
							outFile << "-";
						}	
					}	
					outFile << endl;
				}

				temp = temp->next;
			}	
		}	
	}	
}

//name:			outputSystemTables	
//description:	the system tables are iterated through and their contents, as well as their average, lowest,
//				and highest travel times, are printed to the screen and written to a file
//parameters:	takes in a bool corresponding to whether or not the tables should be written to a file,
//				an ofstream object which is the stream connected to the file, and an int corresponding
//				to which secondary table, either the table holding delay times or the table holding traffic 
//				congestion times, should be displayed in addition to the primary table
//return:		void
void System:: outputSystemTables(bool toFile, ofstream &outFile, int outputCopy)
{
	int index, innerIndex;
	timeSummary();

	if(isEmpty())
	{
		cout << "The system has not tables to output." << endl;
	}	
	else
	{	
		for(index = 0; index < totalCars; index++)
		{
			for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
			{
				if(systemTimeTable[index][innerIndex].name != NULL)
				{
					cout << systemTimeTable[index][innerIndex].name << ":" << systemTimeTable[index][innerIndex].cost << "\t";
				}
			}	
			cout << endl;
		}
		cout << "Lowest travel time: " << lowestTime << endl;	
		cout << "Highest travel time: " << highestTime << endl;	
		cout << "Average travel time: " << averageTravelTime << endl;	
		cout << "Throughput: " << totalCars / averageTravelTime << endl << endl;

		if(toFile)
		{
			for(index = 0; index < totalCars; index++)
			{
				for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
				{
					if(systemTimeTable[index][innerIndex].name != NULL)
					{
						outFile << systemTimeTable[index][innerIndex].name << ":" << systemTimeTable[index][innerIndex].cost << "\t\t\t";
					}
				}	
				outFile << endl;
			}	
			outFile << "Lowest travel time: " << lowestTime << endl;	
			outFile << "Highest travel time: " << highestTime << endl;
			outFile << "Average travel time: " << averageTravelTime << endl;
			outFile << "Throughput: " << totalCars / averageTravelTime << endl << endl;
		}

		if(outputCopy == 1)
		{
			cout << "System Path Table (delay without re-route)" << endl;	
			for(index = 0; index < totalCars; index++)
			{
				for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
				{
					if(systemTimeTableDelay[index][innerIndex].name != NULL)
					{
						cout << systemTimeTableDelay[index][innerIndex].name << ":" << systemTimeTableDelay[index][innerIndex].cost << "\t";
					}
				}	
				cout << endl;
			}
			cout << "Lowest travel time: " << lowestDelayTime << endl;	
			cout << "Highest travel time: " << highestDelayTime << endl;	
			cout << "Average travel time: " << averageTravelTimeDelay << endl;	
			cout << "Throughput: " << totalCars / averageTravelTimeDelay << endl << endl;

			if(toFile)
			{
					outFile << "System Path Table (delay without re-route)" << endl;	
					for(index = 0; index < totalCars; index++)
					{
						for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
						{
							if(systemTimeTableDelay[index][innerIndex].name != NULL)
							{
								outFile << systemTimeTableDelay[index][innerIndex].name << ":" << systemTimeTableDelay[index][innerIndex].cost << "\t\t\t";
							}
						}	
						outFile << endl;
					}	
					outFile << "Lowest travel time: " << lowestDelayTime << endl;	
					outFile << "Highest travel time: " << highestDelayTime << endl;	
					outFile << "Average travel time: " << averageTravelTimeDelay << endl;
					outFile << "Throughput: " << totalCars / averageTravelTimeDelay << endl << endl;
			}	
		}
		if(outputCopy == 2)
		{
			cout << "System Path Table (congestion, with correction)" << endl;
			for(index = 0; index < totalCars; index++)
			{
				for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
				{
					if(systemTimeTableCongestion[index][innerIndex].name != NULL)
					{
						cout << systemTimeTableCongestion[index][innerIndex].name << ":" << systemTimeTableCongestion[index][innerIndex].cost << "\t";
					}
				}	
				cout << endl;
			}
			cout << "Lowest travel time: " << lowestCongestionTime << endl;	
			cout << "Highest travel time: " << highestCongestionTime << endl;	
			cout << "Average travel time: " << averageTravelTimeCongestion << endl;	
			cout << "Throughput: " << totalCars / averageTravelTimeCongestion << endl << endl;	

			if(toFile)
			{
				outFile << "System Path Table (congestion, with correction)" << endl;	
				for(index = 0; index < totalCars; index++)
				{
					for(innerIndex = 0; innerIndex < maxPathLength; innerIndex++)
					{
						if(systemTimeTableCongestion[index][innerIndex].name != NULL)
						{
							outFile << systemTimeTableCongestion[index][innerIndex].name << ":" << systemTimeTableCongestion[index][innerIndex].cost << "\t\t\t";
						}
					}	
					outFile << endl;
				}	
				outFile << "Lowest travel time: " << lowestCongestionTime << endl;	
				outFile << "Highest travel time: " << highestCongestionTime << endl;
				outFile << "Average travel time: " << averageTravelTimeCongestion << endl;
				outFile << "Throughput: " << totalCars / averageTravelTimeCongestion << endl << endl;
			}	
		}	
	}	
}

//name:			errorControl	
//description:	takes in an error code and prints to the screen the type of error encountered
//parameters:	takes in an int corresponding to the error code to be reported
//return:		void
void System:: errorControl(int errorCode)
{
	switch(errorCode)
	{
		case NONEXISTENT_INTERSECTION:

			cout << "error: This intersection does not exist" << endl;

		break;

		case MEMORY_FAULT:

			cout << "error: There is insufficient system memory" << endl;

		break;

		case INTERSECTION_EXISTS:

			cout << "error: the intersection being created already exists" << endl;

		break;	
				
		case NOT_FOUND:

			cout << "error: either intersection connected by this road does not exist" << endl;

		break;

		case MAX_ROADS_EXCEEDED:

			cout << "error: either intersection has already reached the maximum number of roads" << endl;

		break;

		case ZERO_TIME:

			cout << "error: the time entered for this road is zero" << endl;

		break;	
				
		case ROAD_EXISTS:

			cout << "error: a road between these intersections already exists" << endl;

		break;

		case DISCONNECTED:

			cout << "error: the system map contains disconnected components" << endl;

		break;

		case SYSTEM_EMPTY:

			cout << "The system map contains no intersections or roads." << endl;

		break;		

		case NONEXISTENT_DESTINATION:

			cout << "error: one or more of the destination intersections do not exist" << endl;

		break;		

		case INVALID_DESTINATION:

			cout << "error: invalid destination" << endl;

		break;	
	}
}

#endif
