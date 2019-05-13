
all: mtbbb

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
MTBBB_OBJS = mtbbb.o

$(CMN_OBJS) $(MTBBB_OBJS) : $(HDRS)

mtbbb: $(CMN_OBJS) $(MTBBB_OBJS)
	$(CXX) -Wall -std=c++14 -pedantic $(pkg-config --cflags --libs libgps) -o $@ $^ -lm -lwiringPi

# 'make test_3d' will give you a test_3d that is controlled via the keyboard rather
# than by moving the MPU6050.  Use the keys x, X, y, Y, z, Z, and q to exit.
# Note it is the terminal you invoked the binary from that is listening for the
# keyboard, not the window with the wireframe in it, so make sure the terminal
# has input focus.

clean:
	rm -f $(CMN_OBJS) $(MTBBB_OBJS) $(D3D_OBJS) mtbbb
