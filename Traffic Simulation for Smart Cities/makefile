all: SDNCity

SDNCity: main.cpp System.o Intersection.o
	g++ main.cpp System.o Intersection.o -o SDNCity

System.o: System.cpp System.h
	g++ -c System.cpp

Intersection.o: Intersection.cpp Intersection.h
	g++ -c Intersection.cpp

clean:
	rm -f *.o main
