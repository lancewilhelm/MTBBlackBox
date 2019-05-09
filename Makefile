
all: demo_raw demo_dmp

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
DMP_OBJS = demo_dmp.o
RAW_OBJS = demo_raw.o

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

demo_raw: $(CMN_OBJS) $(RAW_OBJS)
	$(CXX) -o $@ $^ -lm

demo_dmp: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm

# 'make test_3d' will give you a test_3d that is controlled via the keyboard rather
# than by moving the MPU6050.  Use the keys x, X, y, Y, z, Z, and q to exit.
# Note it is the terminal you invoked the binary from that is listening for the
# keyboard, not the window with the wireframe in it, so make sure the terminal
# has input focus.

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS) demo_raw demo_dmp