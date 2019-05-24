# Basic Variables
CC = g++
CFLAGS= -c -Wall -O2
LDFLAGS =

# List of Sources
SOURCES = mtbbb.cpp I2Cdev.cpp MPU6050.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Name of Target
EXECUTABLE = mtbbb

CFLAGS += 'pkg-config --cflags -libgps'
LDFLAGS += 'pkg-config --libs libgps'

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(OBJECTS): $(SOURCES)
	$(CC) -c $(SOURCES)
 
clean:
	rm $(OBJECTS) $(EXECUTABLE)

	#g++ -Wall -std=c++14 -pedantic $(pkg-config --libs libgps) -o mtbbb I2Cdev.o MPU6050.o mtbbb.o -lm -lwiringPi -loled96 -lpthread
