# Basic Variables
CC = g++
CFLAGS=-c -Wall -O2
LDFLAGS =

# List of Sources
SOURCES = mtbbb.cpp I2Cdev.cpp MPU6050.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Name of Target
EXECUTABLE = mtbbb

CFLAGS += 'pkg-config --cflags -libgps'
LDFLAGS += 'pkg-config --libs libgps'

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
$(CC) $(CFLAGS) $< -o $@
 
clean:
rm $(OBJECTS) $(EXECUTABLE)

	#g++ -Wall -std=c++14 -pedantic $(pkg-config --libs libgps) -o mtbbb I2Cdev.o MPU6050.o mtbbb.o -lm -lwiringPi -loled96 -lpthread
