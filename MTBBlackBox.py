#!/usr/bin/python
import math
import time
import datetime
import subprocess as sp
import csv
import sys
import argparse

# Put this in a try catch for running on the PC. No need for smbus on here.
try:
    import smbus
except:
    pass

# Define functions for reading sensor data
def read_byte(reg, address, bus):
    return bus.read_byte_data(address, reg)

def read_word(reg, address, bus):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg, address, bus):
    val = read_word(reg, address, bus)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

# Define functions for the math
def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

# Main Function
def main(args):

    # import the args passed from the Starter. Not sure if this is necessary
    args = args

    # Variables
    rot_matrix = []
    t = 0       # start time
    dt = 0.05    # time step, directly controls the time.sleep call
    outputCSVFilename = "data/rot_out_" + datetime.datetime.now().strftime("%c") + ".csv"

    if(args.d == "n"):
        # Housekeeping for the mpu6050 sensor. I2C things
        bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
        address = 0x68       # via i2cdetect
        power_mgmt_1 = 0x6b  # register

    # Creat a new CSV for this run
    with open(outputCSVFilename, 'w') as f:
        f.close()

    print "Running..."

    try:
        while True:

            # Activate to be able to address the module
            # Put in an if statement for computer debugging
            if(args.d == "n"):
                bus.write_byte_data(address, power_mgmt_1, 0)

                gryo_xout = read_word_2c(0x43, address, bus)
                gryo_yout = read_word_2c(0x45, address, bus)
                gryo_zout = read_word_2c(0x47, address, bus)

                acc_xout = read_word_2c(0x3b, address, bus)
                acc_yout = read_word_2c(0x3d, address, bus)
                acc_zout = read_word_2c(0x3f, address, bus)
            else:
                gryo_xout = 0
                gryo_yout = 0
                gryo_zout = 0

                acc_xout = 0
                acc_yout = 0
                acc_zout = 0

            acc_xout_scaled = acc_xout / 16384.0
            acc_yout_scaled = acc_yout / 16384.0
            acc_zout_scaled = acc_zout / 16384.0

            x_rot_temp = get_x_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled)
            y_rot_temp = get_y_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled)
            row_temp = [("%.1f" % t), ("%.2f" % x_rot_temp), ("%.2f" % y_rot_temp)]

            if(args.o == "y"):
                # Print out all of the data on screen if the user wants to
                print "gryo"
                print "--------"
                print "gryo_xout: ", ("%5d" % gryo_xout), " scaled: ", (gryo_xout / 131)
                print "gryo_yout: ", ("%5d" % gryo_yout), " scaled: ", (gryo_yout / 131)
                print "gryo_zout: ", ("%5d" % gryo_zout), " scaled: ", (gryo_zout / 131)
                print "acc"
                print "---------------------"
                print "acc_xout: ", ("%6d" % acc_xout), " scaled: ", acc_xout_scaled
                print "acc_yout: ", ("%6d" % acc_yout), " scaled: ", acc_yout_scaled
                print "acc_zout: ", ("%6d" % acc_zout), " scaled: ", acc_zout_scaled
                print "rot"
                print "---------------------"
                print "X Rotation: " , ("%.2f" % x_rot_temp)
                print "Y Rotation: " , ("%.2f" % y_rot_temp)

                tmp = sp.call('clear', shell=True)  # clears the screen

            # Open the CSV file and write the data
            with open(outputCSVFilename, 'a') as f:
                        writer = csv.writer(f)
                        writer.writerow(row_temp)
            f.close()

            # Pause and advance the time tracker
            time.sleep(dt)
            t += dt

    # If we interupt with the keyboard, print something to recognize it
    except KeyboardInterrupt:
        print "Done"

# Call the main funciton
if __name__ == "__main__":
    main()
