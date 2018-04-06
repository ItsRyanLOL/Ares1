/******** COmpass Info ***************
   ----Calibration Complete----
   Final Offsets: X = -0.22  Y = 0.33
   Final Scales: X = 1.24  Y = 0.84
*/

#define BRAKEVCC 0
#define CW   1 //clockwise
#define CCW  2 //counterclockwise
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

//Debugging Pins
const int statpin = 13; // Pin to enable motors (High = motors Off)



/******* Navigation Varriables ************/
const int interval = 500; //interval at which to check surroundings (milliseconds)
const int headingThreashold = 10; //Threashold before robot executes heading correction (Degrees)
const int motorStateChangeDelay = 50;

int currentHeading; //current direction of robot
int desiredHeading; //desired direction of robot
int distanceToObstacle; //Distance to obstacle

/******* Timer Varriables ********/
unsigned long currentTime; //used for storing P-on time for sketch
unsigned long lastCheckTime = 0; // Last time sensors were checked for obstacles, starts at 0
unsigned long nextNavCheck = 0; //Timer var for nav updates

const int minObstacleDistance = 2.54 * 24; //sets obstance distance to avoid in inches

/********* Debugging **********/
bool verboseDebug = true; //have a verbose serial debug option
bool enableDrive = false;  // be able to turn off motors for sensor debug

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
  currentHeading = getHeading(); // update our current heading
  updateDesiredHeading(); //update our desired heading

  /******** Locomotive Code ***********/
  analogValue = analogRead(sonicSensor);
  if (int(analogValue) < minObstacleDistance) {
    //Print Measurement to detected object in inches on LCD
    if (desiredHeading != -1) matrix.print(int(analogValue / 2.54));
    else matrix.print(char("E"));
    matrix.writeDisplay();
    allStop();
    obstacleTurn();
  }
  else {
    //Print desired heading on LCD if no obstacle detected
    matrix.print(int(desiredHeading));
    matrix.writeDisplay();
    if (currentTime >= nextNavCheck) rightTurn(desiredHeading);
    motorStart();
  }
  /********************************/
  if (currentTime >= (lastCheckTime + interval)) {   //Check for obstacles if time since last check > interval
    lastCheckTime = currentTime; //update last check time

    if (obstacleCheck()) { //Check for obstacles

      if (verboseDebug) {
        Serial.print("Object detected at ");
        Serial.println(distanceToObstacle);
      }
      obstacleTurn(); //Begin evasive actions
    }

  }





}




////////////////Initalize MotoShield Pins ////////////////
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

/*********** Check for shit in our way, return true if found ********************/
bool  obstacleCheck() {
  int sensorRead = analogRead(sonicSensor); //sets local var to ADC value
  if (sensorRead <= minObstacleDistance) return true; //return true if it sees something
  else return false; //path is clear

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
  nextNavCheck = millis() + motorStateChangeDelay; //pause nav updates
}

void rightTurn(int heading) {
  allStop(); //make sure all motors are off
  motorGo(0, CCW, 1023);
  motorGo(1, CW, 1023);
  while (!(getHeading() > (heading - 10) && getHeading() < (heading + 10))) {
    matrix.print(int(getHeading()));
    matrix.writeDisplay();
    delay(5);
  }
  allStop();
  nextNavCheck = millis() + motorStateChangeDelay; //Pause Nav updates
}
////////////////turn to avoid obstacle////////////////
void obstacleTurn() {
  int orignalHeading = getHeading();
  // TODO: determine direction of heading that is closer to beacon

  //change heading
  rightTurn(currentHeading + 70);
  // check for crap and drive if clear
  if (!obstacleDetected()) {
    currentTime = millis(); // update time
    int forwardTime = currentTime + 500;
    //drive forward a bit
    while (!obstacleDetected() && currentTime < forwardTime) {
      motorStart();
      currentTime = millis(); //update time
      delay(5); //slow this train wreck down a little
    }
    allStop(); //stop avoidance
  }
  else { //We turned and crap is STILL THERE!
    rightTurn(findInverseAngle(orignalHeading)); //turn 180 degrees and check again
    if (!obstacleDetected()) {
      currentTime = millis(); // update time
      int forwardTime = currentTime + 500;
      //drive forward a bit
      while (!obstacleDetected() && currentTime < forwardTime) {
        motorStart();
        currentTime = millis(); //update time
        delay(5); //slow this train wreck down a little
      }
      allStop(); //stop avoidance
    }
  }
  // turn towards beacon 70 deg

  //check for obstacles, if found turn 70 deg in other direction and recheck

  //drive forward for ~6 feet in while loop whilst checking for obstacles (A great oppertunity for recursiuon!)

  //
  //delay(1000);
  // find beacon heading
  // break; Only used in loops or switch
}

//********* turn towards beacon when called **********
void beaconTurn(int heading) { //heading is diseried direction
  while (desiredHeading > 180) {
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
  if(tempA >= 0 && tempA < 360) {
    return tempA; 
  }
  else if (tempA < 0) {
    
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
  float declinationAngle = 8.483; //Declination
  // Calibration parameters for accuracy

  float Xoffset = -0.22;
  float Yoffset = 0.33;
  float Xscale = 1.24;
  float Yscale = 0.84;
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
  if (verboseDebug) Serial.print(F("Heading: ")); Serial.println(heading, 2);
}

/********* Gets desired robot heading **********/
void updateDesiredHeading() {
  int receivedData[2];
  if (verboseDebug) {
    Serial.println(F("heading update Requested"));
    //delay(1000);
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
    desiredHeading = receivedData[1]; // Update desiredHeading to direction we want to drive
  }
  else if (receivedData[0] == 1) desiredHeading = 180 + receivedData[1]; //Add 180 because of checksum

  if (desiredHeading == 360) desiredHeading = 0;
  if (verboseDebug) {
    Serial.print(F("Desired direction to drive: "));
    Serial.println(desiredHeading);
  }
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
  motorGo(0, BRAKEGND, 1023);
  motorGo(1, BRAKEGND, 1023);
}

