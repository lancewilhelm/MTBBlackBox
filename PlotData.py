import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv
import sys

def main():
    # Data for plotting
    t = []
    x = []
    y = []

    # Import the Data
    fileLocation = sys.argv[1]

    try:
        with open(fileLocation,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            for row in plots:
                t.append(float(row[0]))
                x.append(float(row[1]))
                y.append(float(row[2]))
    except:
        print "Error has occured"

    plt.plot(t, x, label="x_rot")
    plt.plot(t, y, label="y_rot")

    plt.xlabel('t')
    plt.ylabel('rot')
    plt.legend()
    plt.grid()
    plt.savefig("data/plot.png")
    plt.show()

    print "Plot Created"

# Call the main funciton
if __name__ == "__main__":
    main()
