import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
import argparse

def main(fileLocation):

    # Data for plotting
    t = []
    gyroX = []
    gyroY = []
    gyroZ = []
    accX = []
    accY = []
    accZ = []
    rotX = []
    rotY = []

    try:
        with open(fileLocation,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            for row in plots:
                t.append(float(row[0]))
                gyroX.append(float(row[1]))
                gyroY.append(float(row[2]))
                gyroZ.append(float(row[3]))
                accX.append(float(row[4]))
                accY.append(float(row[5]))
                accZ.append(float(row[6]))
                rotX.append(float(row[7]))
                rotY.append(float(row[8]))
    except:
        print "Error has occured"

    plt.subplot(3,1,1)
    plt.plot(t, rotX, label="x")
    plt.plot(t, rotY, label="y")
    plt.ylabel('Rotation (Degrees)')
    plt.legend()
    plt.grid()

    plt.subplot(3,1,2)
    plt.plot(t, gyroX, label="x")
    plt.plot(t, gyroY, label="y")
    plt.plot(t, gyroZ, label="z")
    plt.ylabel('Gyro (deg/s)')
    plt.legend()
    plt.grid()

    plt.subplot(3,1,3)
    plt.plot(t, accX, label="x")
    plt.plot(t, accY, label="y")
    plt.plot(t, accZ, label="z")
    plt.ylabel('Acc (G)')
    plt.xlabel('t')
    plt.legend()
    plt.grid()

    plt.savefig("data/plot.png")

    print "Plot Created"

# Call the main funciton if ran from standalone
if __name__ == "__main__":

    # Import the Data
    fileLocation = sys.argv[1]

    main(fileLocation)
