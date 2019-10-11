
//System.h
//Authors:
//			Bryan Kline
//			Ryan Lieu
//			Robert Watkins
//December 6, 2017
//CPE400 Project - #6 Software Defined Network for Traffic Management

#include <fstream>
#include "Intersection.h"

#ifndef SYSTEM_H
#define SYSTEM_H

using namespace std;

class System
{
	static const int MAX_INTERSECTIONS = 2;
	static const int MAX_ROADS = 4;
	static const int NONEXISTENT_INTERSECTION = -1;
	static const int SUCCESES = 0;
	static const int MEMORY_FAULT = 1;
	static const int INTERSECTION_EXISTS = 2;
	static const int NOT_FOUND = 3;
	static const int MAX_ROADS_EXCEEDED = 4;
	static const int ZERO_TIME = 5;
	static const int ROAD_EXISTS = 6;
	static const int DISCONNECTED = 7;
	static const int SYSTEM_EMPTY = 8;
	static const int NONEXISTENT_DESTINATION = 9;
	static const int INVALID_DESTINATION = 10;
	static const int INFINITY = 1000000000;

	public:
		System();
		~System();

		bool isEmpty();
		int addIntersection(char* newIntersection, int cars);	
		int addRoad(char* toIntersection, char* fromIntersection, int time);
		int setCarTrajectories(char* fromIntersection, char* destinationList[]);
		int checkCarTrajectories(char* fromIntersection, char* destinationList[]);
		int addDelay(char* fromIntersection, int delay);
		int getTotalCars(char* intersectionName);
		int getSystemCars();
		Intersection* getIntersection(char* intersectionName);

		void OSPF();
		void initializeOSPFTable(char* intersectionName);
		void runDijkstra(char* intersectionName);
		void calculatePaths();
		int getIndex(char* intersectionName, char* label);
		int getTime(char* fromIntersection, char* toIntersection);


		void initializeSystemTable();
		void applyDelay(char* fromIntersection, int delay);
		void simulateTraffic();
		void combineCars();
		void timeSummary();
		
		void displaySystem(bool toFile, ofstream &outFile);
		void outputSystemTables(bool toFile, ofstream &outFile, int outputCopy);
		void errorControl(int errorCode);

	friend class Intersection;	

	private:	
		int intersectionCount;
		int totalCars;
		int maxPathLength;
		int dijkstraRun;
		int latestTime;
		int threshold;

		int lowestTime;
		int lowestDelayTime;
		int lowestCongestionTime;
		int highestTime;
		int highestDelayTime;
		int highestCongestionTime;
		float averageTravelTime;
		float averageTravelTimeDelay;
		float averageTravelTimeCongestion;

		struct Path** systemTimeTable;
		struct Path** systemTimeTableDelay;
		struct Path** systemTimeTableCongestion;

		Intersection* intersectionList;
};

#endif
