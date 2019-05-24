# Basic Variables
CC = g++
CFLAGS= -Wall -O2
LDFLAGS = -lm -lwiringPi -loled96 -lpthread

# List of Sources
SOURCES = mtbbb.cpp I2Cdev.cpp MPU6050.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Name of Target
EXECUTABLE = mtbbb

# Pkg Config
PKG = `pkg-config --cflags --libs libgps`

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) $(PKG) $(LDFLAGS) $(OBJECTS) -o $@

$(OBJECTS): $(SOURCES)
	$(CC) -c $(CFLAGS) $(SOURCES)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

	#g++ -Wall -std=c++14 -pedantic $(pkg-config --libs libgps) -o mtbbb I2Cdev.o MPU6050.o mtbbb.o -lm -lwiringPi -loled96 -lpthread
