#define BRAKEVCC 0
#define CW   2 //clockwise
#define CCW  1 //counterclockwise
#define BRAKEGND 3
#define CS_THRESHOLD 100



/******* For 9DOF Stick **********/
#include <SparkFunLSM9DS1.h>
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
LSM9DS1 compass;  // Storing the features in the HMC5883 library
/*********************************/

/******* For Fio *****************/
#include <Wire.h> // I2C Libary
const int xbeeWireAddress = 8; //I2C adress of slave xbee
/*********************************/

/******* For LCD Display *********/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
Adafruit_7segment matrix = Adafruit_7segment();
int analogValue;
/*********************************/

/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */

/************ Pin Definitions ********************/
// H-Bridge Pins
const int inApin[2] = {7, 4};  // INA: Clockwise input
const int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
const int pwmpin[2] = {5, 6}; // PWM input
const int cspin[2] = {A2, A3}; // CS: Current sense ANALOG input
const int enpin[2] = {A0, A1}; // EN: Status of switches output (Analog pin)

// Sensor Pins
const int sonicSensor = A15; //Ultrasonic sensor pin

//Bump Sensor Pin and variable
const int bumpPin = 2; // bumper pin 2
int bumpVar = 0;          // Var for bump

//Debugging Pins
const int statpin = 13; // Pin to enable motors (High = motors Off)



/******* Navigation Varriables ************/
const int interval = 250; //interval at which to check surroundings (milliseconds)
const int headingThreashold = 10; //Threashold before robot executes heading correction (Degrees)
const int motorStateChangeDelay = 50;

int currentHeading; //current direction of robot
int driveDirection = 180;                                               /**************Changed to Compass**************/
int desiredHeading = driveDirection; //desired direction of robot
int distanceToObstacle; //Distance to obstacle



/******* Timer Varriables ********/
unsigned long currentTime; //used for storing P-on time for sketch
unsigned long lastCheckTime = 0; // Last time sensors were checked for obstacles, starts at 0
unsigned long nextNavCheck = 0; //Timer var for nav updates
unsigned long ddDisp = 0; // TImer for disered heading Display updates
bool dispState = true;

const int minObstacleDistance = 2.54 * 14; //sets obstance distance to avoid in inches

/********* Debugging **********/
bool verboseDebug = true; //have a verbose serial debug option
bool enableDrive = true;  // be able to turn off motors for sensor debug

void setup()
{
  Serial.begin(9600);

  matrix.begin(0x70); //Start LCD Display
  compass.begin(); //Start the 9DOF stick

  initalizeMotoShieldPins(); // setup H-bridge pins
  pinMode(sonicSensor, INPUT); // Initalize sonic sensor pin
  currentTime = millis(); //Start keeping track of time

  delay(5000); //give some time to clear fingers
}


void loop() {
  currentTime = millis(); //update P-on time, should be first action of every loop
  currentHeading = int(getHeading()); // update our current heading
  analogValue = analogRead(A15);

  if (currentTime >= nextNavCheck) {
    nextNavCheck += (currentTime+500);
    driveDirection = updateDesiredHeading();
    Serial.print("Update on heading: ");
    Serial.println(int(driveDirection));
  }

  /******** Locomotive Code ***********/
  //Print desired heading on LCD if no obstacle detected
  if (ddDisp <= currentTime) {
    bool tempS = dispState;
    dispState = !tempS;
    ddDisp += 2000;
    matrix.writeDisplay();
  }
  if(dispState) {
    matrix.print(int(driveDirection));
  }
  if (!dispState) {
    matrix.print(int(currentHeading));
  }
  /*if (verboseDebug) {
    Serial.print("Current heading: ");
    Serial.println(currentHeading);
  }*/
  if (int(analogValue) < minObstacleDistance) {
    if (verboseDebug) {
      Serial.print("Sensor: ");
      Serial.print(analogValue);
      Serial.print("cm");
    }
    obstacleTurn(); // GO around shit
  }

  if (driveDirection < currentHeading) slowLeft();
  else slowRight();
  matrix.print(currentHeading);
  matrix.writeDisplay();

}




////////////////Initalize MotoShield Pins ////////////////
void slowRight() {
  motorGo(0, CCW, 125);
  motorGo(1, CW, 255);
  //Serial.println("Going Left");
}
void slowLeft() {
  motorGo(1, CW, 125);
  motorGo(0, CCW, 255);
  //Serial.println("Going Right");
}
void initalizeMotoShieldPins() {
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  if (enableDrive) {


    for (int i = 0; i < 2; i++)
    {
      digitalWrite(inApin[i], LOW);
      digitalWrite(inBpin[i], LOW);
    }
  }
}


/*****************************************************
          Motor Control Functions
 ****************************************************/

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}


void motorStart() {
  //starts motor in forward direction
  motorGo(0, CW, 1023);
  motorGo(1, CW, 1023);
  nextNavCheck = millis() + motorStateChangeDelay; // Pause nav updates
}



//********* turn towards beacon when called **********
void beaconTurn(int heading) { //heading is diseried direction
  while (desiredHeading > driveDirection) {
    for (currentHeading; currentHeading < desiredHeading; currentHeading += 5) {
      motorGo(CCW, 0, 1023);
      motorGo(CW, 1, 1023);
    }
    motorGo(CCW, 0, 0);
    motorGo(CW, 1, 0);
  }
  while (desiredHeading <= -180) {
    for (currentHeading; currentHeading < desiredHeading; currentHeading -= 5) {
      motorGo(CW, 0, 1023);
      motorGo(CCW, 1, 1023);
    }
    motorGo(CW, 0, 0);
    motorGo(CCW, 1, 0);
  }
}
int findInverseAngle(int angle) {
  if (angle >= 180) return (angle - 180);
  else if (angle < 180) return (angle + 180);
}
int addAngle (int currentAngle, int add) {
  int tempA = (currentAngle + add);
  if (tempA >= 0 && tempA < 360) {
    return tempA;
  }
  else if (tempA < 0) {
    return (360 + tempA);
  }
  else if (tempA > 360) {
    return (tempA - 360);
  }
}
/********* Returns Robot's current Heading *********/
float getHeading() {

  float x, y;

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/ Mine is:
  // +8° 29' West, which is 8.483 Degrees, or (which we need) 0.14805628 radians, I will use 8.483
  // degrees since we convert it to Radians later in the code.
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off
  float declinationAngle = 8.4; //Declination
  // Calibration parameters for accuracy

  /* Final Offsets: X = 0.03 Y = 0.27
    Final Scales: X = 1.05  Y = 0.95
  */

  float Xoffset = 0.03;
  float Yoffset = 0.27;
  float Xscale = 1.05;
  float Yscale = 0.95;
  // Get Magnetic field readings
  compass.readMag();

  // Subtract calculated offsets from magnetometer data
  x = compass.calcMag(compass.mx) - Xoffset;
  y = compass.calcMag(compass.my) - Yoffset;

  // Scaling correction
  x *= Xscale;
  y *= Yscale;

  // Begin to calculate heading
  float heading;

  // Calculate the angle
  if (y == 0)
    heading = (x < 0) ? PI : 0;
  else
    heading = atan2(y, x);

  // Correct for Declination
  heading -= declinationAngle * (PI / 180);

  // Correct for sign errors
  if (heading > 2 * PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= (180.0 / PI);

  //Return the heading
  return heading;
}

/********* Gets desired robot heading **********/
int updateDesiredHeading() {
  int receivedData[2];
  if (verboseDebug) {
    Serial.println(F("heading update Requested"));
  }
  Wire.requestFrom(xbeeWireAddress, 2); // Request 2 bytes of data from xbeeWireAddress

  int a = 0; //start counting
  while (Wire.available())   // slave may send less than requested, so let's do something about that
  {
    receivedData[a] = Wire.read();    // receive a byte as an int
    if (verboseDebug) Serial.println(receivedData[a]);        // print the int

    a++; //increment a by 1
  }
  if (receivedData[0] == 2) {
    desiredHeading == -1;
    if (verboseDebug) {
      Serial.println(F("Error Parsing beacon data."));
    }
    return;
  }
  else if (receivedData[0] == 0) {
    return receivedData[1]; // Update desiredHeading to direction we want to drive
  }
  else if (receivedData[0] == 1) return (180 + receivedData[1]); //Add 180 because of checksum

  if (desiredHeading == 360) return 0;
  }
void compassSetup() {

  compass.settings.device.commInterface = IMU_MODE_I2C;
  compass.settings.device.mAddress = LSM9DS1_M;
  compass.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // compass.begin(), which verifies communication with the compass
  // and turns it on.
  if (!compass.begin())
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
}
void allStop() {
  motorGo(0, BRAKEGND, 254);
  motorGo(1, BRAKEGND, 254);
}

void obstacleTurn() {
  Serial.println("FUUUUUUUUUUUUUUUUUUUUUCKKKKKKK SHITS IN THE WAI BRUH");
  matrix.print(int(analogRead(A15) / 2.54));
  matrix.writeDisplay();
  while(analogRead(A15) < minObstacleDistance) {
    avoidLeft();
  }
      avoidForward();
}
void avoidRight() {
  motorGo(0, CW, 254);
  motorGo(1, CW, 254);
}
void avoidLeft() {
  motorGo(0, CCW, 254);
  motorGo(1, CCW, 254);
}
void avoidForward() {
  motorGo(0, CCW, 254);
  motorGo(1, CW, 254);
}

