#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
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
#include <csignal>
#include "oled96.h"
#include <vector>

// Define for the LEDS
#define GREEN 0
#define RED 1
#define BUTTON 4

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

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
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// gps stuff
bool gpsfail = false;
bool gpsLock = false;
bool newGPSData = false;
int fifoOverflow = 0;

//stuff for screen
float maxSpeed = 0;
std::string maxSpeedStr;
int iChannel = 1;	// I2C bus 1
int iOLEDAddr = 0x3c; // typical address; it can also be 0x3d
int iOLEDType = OLED_128x64; // Change this for your specific display
int bFlip = 0;
int bInvert = 0;

//stuff for continuous running
bool runLoop = true;

//device default orrientation offsets
float pitchOffset = 27.21; // deg
float rollOffset = 0; // deg

//jump setup
int numberOfJumps = 0;
float jumpMinThreshold = -2;
float jumpMaxThreshold = 1;
bool possibleJumpEvent = false;
float jumpEventMinTime;
float jumpEventMinVal = 0;
float jumpEventMaxTime;
float jumpEventMaxVal = 0;
float hangtime;
float maxHangtime = 0;
std::string maxHangtimeStr;

// -----------MTBBB Data Structure--------------
struct mtbbbDataStruct {
  float t, yaw, pitch, dpitch, roll, accX, accY, accZ, daccZ, lat, lon, speed, alt, hangtime;
  int jump;
  std::string gpstime;
};

// Initialize the data structure
std::vector<mtbbbDataStruct> mtbbbData;

// Buffer size for derivatives
int bufferSize = 5;
int bufferCenterOffset = bufferSize / 2;

struct jumpNode{
  bool max; // if it's a max node this is true. False for min node.
  float t, accZ;
  jumpNode *last;
};

jumpNode *head = NULL;

void calculateJump(){
  jumpNode *temp = new jumpNode;
  temp = head->last;  //set our first node to the last one from head

  bool complete = false;
  while(!complete){
    if(temp->max){
      jumpEventMaxTime = temp->t;
      jumpEventMaxVal = temp->accZ;
      complete = true;
    } else if (temp == NULL){ //emergency catch
      std::cout << "end of jump list" << std::endl;
      complete = true;
    // } else if (temp->max && temp->accZ < jumpEventMaxVal){
    //   complete = true;
    } else {
      temp = temp->last;
    }
  } // end while(!complete)

  // Increase the jump counter and calculate hangtime
  numberOfJumps += 1;
  hangtime = jumpEventMinTime - jumpEventMaxTime;
  std::cout << "JUMP " << std::to_string(jumpEventMaxVal) << std::endl;

  // Update maxHangtime if necessary
  if (hangtime > maxHangtime){
    maxHangtime = hangtime;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << maxHangtime;
    maxHangtimeStr = stream.str();
  }

  // reset the variables
  jumpEventMinVal = 0;
  jumpEventMaxVal = 0;
  possibleJumpEvent = false;

} // end calculateJump()

void createJumpNode(float time, bool max, std::ofstream &myfile){
  std::cout << "JUMP NODE" << std::endl;
  jumpNode *temp = new jumpNode;
  temp -> t = time;
  temp -> accZ = (static_cast<float>(aaWorld.z) / 4096) - 1;  // minus 1G for gravity
  temp -> max = max;
  temp -> last = head;

  // If we detected a min threshold breach, record it, and it's time and value
  if (temp->max == false){
    possibleJumpEvent = true;
    jumpEventMinTime = temp->t;
    jumpEventMinVal = temp->accZ;
  }

  // set head to temp node
  head = temp;

  // If we just completed a jump then do some calculations
  if (possibleJumpEvent && temp->max == true){
    std::cout << "CALC JUMP" << std::endl;
    calculateJump();
    myfile << numberOfJumps << "," << hangtime << std::endl;
  } else {
    myfile << "," << std::endl;
  }
} // end creatJumpNode()

// ================================================================
// ===                      FUNCTIONS                           ===
// ================================================================

void signalHandler(int signum) {
   std::cout << "Interrupt signal (" << signum << ") received.\n";

   // cleanup and close up stuff here
	 // oledShutdown();

   // terminate program
   exit(signum);
}

bool fileExists (const std::string &name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void setOffsets(){
  bool finished = false;

  oledFill(0x00); // Clear the screen
  oledWriteString(2,0,"GETTING OFFSETS...");

  while(!finished){
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Gather data from dmp
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Set offsets
        pitchOffset = (ypr[1] * 180/M_PI);
        rollOffset = (ypr[2] * 180/M_PI);

        finished = true;
    } //end if(fifocount)
  } //end while(!finished)

  // Display offsets on screen
  std::string pitchOffsetLine = "Pitch: " + std::to_string(pitchOffset);
  std::string rollOffsetLine = "Roll: " + std::to_string(rollOffset);
  oledWriteString(2,2,pitchOffsetLine);
  oledWriteString(2,3,rollOffsetLine);
  delay(5000);
  return;
} //end setOffsets()

// ------------------------ SETUP ----------------------
void setup() {

    // OLED Init
    int i = oledInit(iChannel, iOLEDAddr, iOLEDType, bFlip, bInvert);
    oledFill(0x00); // Clear the screen
    // oledWriteLogo();
    oledWriteString(2,0,"INITIALIZING...");

    // Setup the GPIO stuff for the LEDs and buttons
    fflush(stdout);
    std::cout << "Initializing wiringPi..." << std::endl;
    wiringPiSetup();
    pinMode(GREEN, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BUTTON, INPUT);
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);

    // initialize device
    std::cout << "Initializing I2C devices..." << std::endl;
    mpu.initialize();

    // verify connection
    std::cout << "Testing device connections..." << std::endl;
    std::cout << (mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed") << std::endl;

    // load and configure the DMP
    std::cout << "Initializing DMP..." << std::endl;
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

    // Lowpass filter at 5 Hz
    mpu.setDLPFMode(6);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        std::cout << "Enabling DMP..." << std::endl;
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        std::cout << "DMP ready!" << std::endl;
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        std::cout << "DMP Initialization failed code: " << devStatus << std::endl;
    }

    // if button is still pressed, calibrate offsets
    if(digitalRead(BUTTON) == HIGH){
      setOffsets();
    }
} //end setup()

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(std::ofstream &myfile, std::chrono::high_resolution_clock::time_point &t0, std::chrono::high_resolution_clock::time_point &t1, gpsmm &gps_rec) {

    // Saves some typing and confusion
    using namespace std;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // create a structure for the data
    struct gps_data_t *gpsd_data;

    // Get GPS goodies if setup did not fail and we are not waiting for a packet
    if(!gpsfail && gps_rec.waiting(100)){
      std::cout << "GPS READY" << std::endl;
      newGPSData = true;
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
        std::cout << "FIFO overflow!" << std::endl;

        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, HIGH);

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42 && gpsd_data != NULL) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //mpu.resetFIFO();

        digitalWrite(RED, LOW);
        digitalWrite(GREEN, HIGH);

        //--------------------------Data Acquisition---------------------------
        // Create a new vector entry for data
        mtbbbData.push_back(mtbbbDataStruct());
        int n = mtbbbData.size() - 1; // Gets current count for reference

        // Record the time
        t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = t1 - t0;

        // MPU6050 DMP data acquisition
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        mtbbbData[n].t = duration.count();
        mtbbbData[n].yaw = (ypr[0] * 180/M_PI);
        mtbbbData[n].pitch = ((ypr[1] * 180/M_PI) - pitchOffset);
        mtbbbData[n].roll = ((ypr[2] * 180/M_PI) - rollOffset);
        mtbbbData[n].accX = (static_cast<float>(aaWorld.x) / 4096);
        mtbbbData[n].accY = (static_cast<float>(aaWorld.y) / 4096);
        mtbbbData[n].accZ = (static_cast<float>(aaWorld.z) / 4096) - 1;  // minus 1G for gravity
        mtbbbData[n].dpitch = 0; //temp
        mtbbbData[n].daccZ = 0;  //temp

        if(mtbbbData.size() >= bufferSize){
          mtbbbData[n-bufferCenterOffset].dpitch = (mtbbbData[n].pitch - mtbbbData[n-(bufferSize-1)].pitch)/(mtbbbData[n].t - mtbbbData[n-(bufferSize-1)].t)
          mtbbbData[n-bufferCenterOffset].daccZ = (mtbbbData[n].accZ - mtbbbData[n-(bufferSize-1)].accZ)/(mtbbbData[n].t - mtbbbData[n-(bufferSize-1)].t)
        }

        // GPS data acquisition
        timestamp_t ts { gpsd_data->fix.time };

        // convert GPSD's timestamp_t into time_t
        time_t seconds { (time_t)ts };
        seconds -= 25200; // 7 hour correction for time zone
        auto tm = *std::localtime(&seconds);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
        auto time_str { oss.str() };

        if(newGPSData){
          mtbbbData[n].gpstime = time_str;
          mtbbbData[n].lat = gpsd_data->fix.latitude;
          mtbbbData[n].lon = gpsd_data->fix.longitude;
          mtbbbData[n].speed = gpsd_data->fix.speed * MPS_TO_MPH;
          mtbbbData[n].alt = gpsd_data->fix.altitude * METERS_TO_FEET;
        }

        // Print data to terminal (debugging)
        std::cout << std::fixed << std::setprecision(2) << "ypdr: " << mtbbbData.back().yaw << "," << mtbbbData.back().pitch << "," << mtbbbData.back().dpitch << "," << mtbbbData.back().roll << std::endl;

        // check for new max speed
        // if(mtbbbData[n].speed > maxSpeed){
        //   maxSpeed = mtbbbData[n].speed;
        //   std::ostringstream stream;
        //   stream << std::fixed << std::setprecision(2) << maxSpeed;
        //   maxSpeedStr = stream.str();
        // }

        std::ostringstream osss;
        osss << std::put_time(&tm, "%H:%M:%S");
        auto oled_time_str { osss.str() };

        // Ouput GPS data
        std::setprecision(6);
        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        if(seconds == 0 || newGPSData == false){
          // myfile << "," << "," << "," << "," << ","; // << std::endl;
        } else {
          // myfile << std::setprecision(6) << time_str << "," << latitude << "," << longitude << "," << speed << "," << alt << ","; // << std::endl;
        }

        // // Jump calculations
        // if(fourth->daccZ > 0 && mtbbbData->daccZ <= 0 && mtbbbData->accZ > jumpMaxThreshold){
        //   createJumpNode(duration.count(),true,myfile);  // jump maximum (takeoff)
        // } else if (fourth->daccZ < 0 && mtbbbData->daccZ >= 0 && mtbbbData->accZ < jumpMinThreshold){
        //   createJumpNode(duration.count(),false,myfile); // jump minimum (landing)
        // } else {
        //   myfile << "," << std::endl;
        // }


        // If we have received new GPS data, update the screen (equates to 1Hz screen updates)
        if(newGPSData){ //fix this

          std::string maxSpeedLine = "Max Sp: " + maxSpeedStr;
          // std::string jumpLine = "Jumps: " + std::to_string(numberOfJumps);
          // std::string hangtimeLine = "Max Hang: " + maxHangtimeStr;

          // display current GPS state
          if(seconds == 0){
            oledWriteString(0,0,"GPS NL");
            oledWriteString(13,0,oled_time_str);
          } else {
            oledWriteString(0,0,"GPS   ");
            oledWriteString(13,0,oled_time_str);
            oledWriteString(0,3,maxSpeedLine);
            // oledWriteString(0,4,jumpLine);
            // oledWriteString(0,5,hangtimeLine);
          }

          newGPSData = false;
        } // end if (newGPSData)
    } // end if(fifocount)
} // end loop()

int main() {

    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    // Setup the MPU6050 stuff
    setup();
    usleep(100000);   // Sleep for 0.1 s. This is important I think...

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
    myfile << "t,yaw,pitch,dpitch,roll,aworldX,aworldY,aworldZ,daworldZ,gpstime,lat,lon,speed,alt,jump,hangtime\n";

    // Initialize GPS
    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
      std::cerr << "No GPSD running.\n";
      gpsfail = true;
    }

    // Clear display before starting
    oledFill(0x00);

    //-----------------------------------------------------------
    //                         Main Loop
    //-----------------------------------------------------------
    while(true){
      if (runLoop){
        // Run the main loop
        loop(myfile, t0, t1, gps_rec);

        // Check for button press. End program
        if(digitalRead(BUTTON) == HIGH){
          oledWriteString(2,7,"Ending MTBBB....");

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

          // Check that the data file exists. THIS MAY NEED RETOUCHING
          bool checkFile = fileExists(filename);
          if(checkFile){
            oledWriteString(2,1,"Data file exists!");
          } else {
            oledWriteString(4,1,"NO DATA FILE!");
          }

          // switch runLoop bool
          runLoop = false;

        } // end if(Button==HIGH)
      } else {
        // Check for button press. Start program
        if(digitalRead(BUTTON) == HIGH){
          // Initialize new t0
          t0 = std::chrono::high_resolution_clock::now();

          // Reset Stats
          numberOfJumps = 0;
          hangtime = 0;
          maxHangtime = 0;
          maxHangtimeStr = "";
          maxSpeed = 0;

          // Initialize file for recording
          std::ofstream myfile;

          // Open the file and write header. May want to reconsider file naming
          std::chrono::seconds timestamp = std::chrono::duration_cast< std::chrono::seconds >(std::chrono::system_clock::now().time_since_epoch());
          std::ostringstream os;
          os << "/home/pi/mtbblackbox/data/data-" << timestamp.count() << ".csv";
          std::string filename = os.str();
          myfile.open (filename);
          myfile << "t,yaw,pitch,roll,aworldX,aworldY,aworldZ,gpstime,lat,lon,speed,alt,overflow\n";

          // Clear display before starting
          oledFill(0x00);
          oledWriteString(2,0,"RESTARTING...");

          // switch runLoop bool
          runLoop = true;

          // MANDATORY delay so as to not retrigger program stop immediately
          delay(2000);

          // if button is still pressed, calibrate offsets
          if(digitalRead(BUTTON) == HIGH){
            setOffsets();
          }

          oledFill(0x00); // clear screen
        } // end if(Button==HIGH)
      } // end if(runLoop), else
    } // end while(true)

    return 0;
} // end main()
