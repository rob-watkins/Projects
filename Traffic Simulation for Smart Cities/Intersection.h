
//Intersection.h
//Authors:
//			Bryan Kline
//			Ryan Lieu
//			Robert Watkins
//December 6, 2017
//CPE400 Project - #6 Software Defined Network for Traffic Management

#ifndef INTERSECTION_H
#define INTERSECTION_H

class Intersection;

struct Road
{
	static const int MAX_INTERSECTIONS = 2;

	int time;
 	Intersection* intersections[MAX_INTERSECTIONS];
};

struct Car
{
	static const int MAX_DESTINATIONS = 2;

	char* name;
 	char* trajectory[MAX_DESTINATIONS];

 	struct Path* naiveOSPFPath;	
};

struct Path
{
	int cost;
	bool combine;
	char* name;
	struct Path* next;
};

struct DijkstraNode
{
	static const int INFINITY = 1000000;

	int cost;
	char* through;
	bool done;
};

class Intersection
{
	static const int THIS_INTERSECTION = 0;
	static const int TO_INTERSECTION = 1;
	static const int MAX_ROADS = 4;
	static const int MAX_CARS = 100;

	public:
		Intersection(char* intersectionName, int totalCars);
		~Intersection();

	friend class System;	

	private:
		char* name;
		int carCount;
		int roadCount;
		struct Road connectedRoads[MAX_ROADS];	
		struct Car* cars;
		Intersection* next;
		struct DijkstraNode** OSPFTable;
};

#endif