#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <chrono>
#include <ctime>
#include <wiringPi.h>
#include <libgpsmm.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool gpsfail = false;

int fifoOverflow = 0;

// Define for the LEDS
#define GREEN 0
#define RED 1
#define BUTTON 4

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // Setup the GPIO stuff for the LEDs and buttons
    printf("Initializing wiringPi...\n");
    wiringPiSetup();
    pinMode(GREEN, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BUTTON, INPUT);
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);

    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(-2081);
    mpu.setYAccelOffset(1071);
    mpu.setZAccelOffset(1541);
    mpu.setXGyroOffsetUser(-18);
    mpu.setYGyroOffsetUser(-51);
    mpu.setZGyroOffsetUser(-68);
    mpu.setXGyroOffset(-18);
    mpu.setYGyroOffset(-51);
    mpu.setZGyroOffset(-68);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(std::ofstream &myfile, std::chrono::high_resolution_clock::time_point &t0, std::chrono::high_resolution_clock::time_point &t1, gpsmm &gps_rec) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // create a structure for the data
    struct gps_data_t *gpsd_data;

    // Get GPS goodies if setup did not fail and we are not waiting for a packet
    if(!gpsfail && gps_rec.waiting(1000)){
      std::cout << "GPS READY" << std::endl;
      // Read the GPS data and error check at the same time
      if ((gpsd_data = gps_rec.read()) == NULL) {
        std::cerr << "GPSD READ ERROR.\n";
        gpsfail = true;
      } else if ((gpsd_data->fix.mode < MODE_2D)) {
          std::cout << "RETURNING DUE TO FIX MODE ERR" << std::endl;
      }
    }

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // debugging
    fifoOverflow = fifoCount;

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, HIGH);

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (fifoCount >= 42 && gpsd_data != NULL) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //mpu.resetFIFO();

        digitalWrite(RED, LOW);
        digitalWrite(GREEN, HIGH);

        // Record the time
        t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = t1 - t0;
        myfile << std::setprecision(6) << duration.count() << ",";

        // Gather data from dmp
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        // Yaw Pitch Roll
        printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        myfile << std::fixed << std::setprecision(2) << (ypr[0] * 180/M_PI) << "," << (ypr[1] * 180/M_PI) << "," << (ypr[2] * 180/M_PI) << ",";

        // display real acceleration, adjusted to remove gravity
        printf("areal %6d %6d %6d    ", (static_cast<float>(aaReal.x) / 4096), (static_cast<float>(aaReal.y) / 4096), (static_cast<float>(aaReal.z) / 4096));
        myfile << (static_cast<float>(aaReal.x) / 4096) << "," << (static_cast<float>(aaReal.y) / 4096) << "," << (static_cast<float>(aaReal.z) / 4096) << ",";

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        printf("aworld %6d %6d %6d \n", (static_cast<float>(aaWorld.x) / 4096), (static_cast<float>(aaWorld.y) / 4096), (static_cast<float>(aaWorld.z) / 4096));
        myfile << (static_cast<float>(aaWorld.x) / 4096) << "," << (static_cast<float>(aaWorld.y) / 4096) << "," << (static_cast<float>(aaWorld.z) / 4096) << ",";

        // Display and wriet the GPS data
        timestamp_t ts { gpsd_data->fix.time };
        auto latitude  { gpsd_data->fix.latitude };
        auto longitude { gpsd_data->fix.longitude };
        auto speed     { gpsd_data->fix.speed * MPS_TO_MPH};
        auto alt       { gpsd_data->fix.altitude * METERS_TO_FEET};

        // convert GPSD's timestamp_t into time_t
        time_t seconds { (time_t)ts };
        auto   tm = *std::localtime(&seconds);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
        auto time_str { oss.str() };

        // ouput GPS data
        std::setprecision(6);
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout << "gpsTime: " << time_str << ", Lat: " << latitude << ",  Lon: " << longitude << ", Sp: " << speed << ", Alt: " << alt << std::endl;
        if(seconds == 0){
          myfile << "," << "," << "," << "," << ","; // << std::endl;
          digitalWrite(RED, HIGH);
          digitalWrite(GREEN, HIGH);
        } else {
          myfile << std::setprecision(6) << time_str << "," << latitude << "," << longitude << "," << speed << "," << alt << ","; // << std::endl;
        }

        myfile << fifoOverflow << std::endl;
    }
}

int main() {
    // Setup the MPU6050 stuff
    setup();
    usleep(100000);   // This is important I think...

    // Start the clock for time purposes
    std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
    t0 = std::chrono::high_resolution_clock::now();

    // Initialize file for recording
    std::ofstream myfile;

    // Open the file
    std::chrono::seconds timestamp = std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch());
    std::ostringstream os;
    os << "/home/pi/mtbblackbox/data/data-" << timestamp.count() << ".csv";
    std::string filename = os.str();
    myfile.open (filename);
    myfile << "t,yaw,pitch,roll,arealX,arealY,arealZ,aworldX,aworldY,aworldZ,gpstime,lat,lon,speed,alt,overflow\n";

    // Initialize GPS
    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
      std::cerr << "No GPSD running.\n";
      gpsfail = true;
    }

    for (;;){
      // Run the main loop
      loop(myfile, t0, t1, gps_rec);

      // Check for button press. Exit loop if pressed
      if(digitalRead(BUTTON) == HIGH){
        break;
      }
    }

    // Close the file
    myfile.close();

    // Signal the end of the program
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    delay(500);
    digitalWrite(RED, LOW);
    delay(500);
    digitalWrite(RED, HIGH);
    delay(500);
    digitalWrite(RED, LOW);

    return 0;
}
