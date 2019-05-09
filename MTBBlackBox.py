#!/usr/bin/python
import math
import time
from datetime import datetime, timedelta
import subprocess as sp
import csv
import sys
import argparse
from MPU6050 import MPU6050

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -5489
y_accel_offset = -1441
z_accel_offset = 1305
x_gyro_offset = -2
y_gyro_offset = -72
z_gyro_offset = -5
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)

count = 0
FIFO_buffer = [0]*64

FIFO_count_list = list()

# Main Function
def main(args):

    # import the args passed from the Starter. Not sure if this is necessary
    args = args

    # Variables
    t = 0       # start time
    dt = 0.001    # time step, directly controls the time.sleep call
    outputCSVFilename = "data/" + datetime.now().strftime("%c") + ".csv"

    # Creat a new CSV for this run and write the header row
    with open(outputCSVFilename, 'w') as f:
        first_row = ["t", "roll", "pitch", "yaw", "accX", "accY", "accZ"]
        writer = csv.writer(f)
        writer.writerow(first_row)
        f.close()

    print("Running...")
    startTime = float(datetime.now().timestamp())

    try:
        while True:

            FIFO_count = mpu.get_FIFO_count()
            mpu_int_status = mpu.get_int_status()

            # If overflow is detected by status or fifo count we want to reset
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                mpu.reset_FIFO()
                print('overflow!')

            # Check if fifo data is ready
            elif (mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                while FIFO_count < packet_size:
                    FIFO_count = mpu.get_FIFO_count()
                FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
                accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = mpu.DMP_get_gravity(quat)
                linearAccel = mpu.DMP_get_linear_accel(accel, grav)
                roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                tmp = sp.call('clear', shell=True)  # clears the screen
                print('roll: ' + str(roll_pitch_yaw.x))
                print('pitch: ' + str(roll_pitch_yaw.y))
                print('yaw: ' + str(roll_pitch_yaw.z))
                print('accX: ' + str(linearAccel.x))
                print('accY: ' + str(linearAccel.y))
                print('accZ: ' + str(linearAccel.z))

                nowTime = float(datetime.now().timestamp())
                t = (nowTime - startTime)
                full_row_temp = [("%.4f" % t), ("%.4f" % roll_pitch_yaw.x), ("%.4f" % roll_pitch_yaw.y), ("%.4f" % roll_pitch_yaw.z), ("%.4f" % linearAccel.x), ("%.4f" % linearAccel.y), ("%.4f" % linearAccel.z)]

            # Open the CSV file and write the data
            with open(outputCSVFilename, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(full_row_temp)
            f.close()

            # Pause and advance the time tracker
            #time.sleep(dt)
            #t += dt

    # If we interupt with the keyboard, print something to recognize it
    except KeyboardInterrupt:
        print("Done")

# Call the main funciton
if __name__ == "__main__":
    main()
