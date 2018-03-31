#define BRAKEVCC 0
#define CW   1 //clockwise
#define CCW  2 //counterclockwise
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */

/************ Pin Definitions ********************/
// H-Bridge Pins
const int inApin[2] = {7, 4};  // INA: Clockwise input
const int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
const int pwmpin[2] = {5, 6}; // PWM input
const int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
const int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

// Compass pins

// Sensor Pins
const int sonicSensor = A0; //Ultrasonic sensor pin

//Debugging Pins
const int statpin = 13; // Pin to enable motors (High = motors Off)



/******* Navigation Varriables ************/
const int interval = 500; //interval at which to check surroundings (milliseconds)
const int headingThreashold = 10; //Threashold before robot executes heading correction (Degrees)

int currentHeading; //current direction of robot
int desiredHeading; //desired direction of robot
int distanceToObstacle; //Distance to obstacle

/******* Timer Varriables ********/
unsigned long currentTime; //used for storing P-on time for sketch
unsigned long lastCheckTime = 0; // Last time sensors were checked for obstacles, starts at 0

const int obstacleDistance = 2.54 * 24; //sets obstance distance to avoid in inches

/********* Debugging **********/
bool verboseDebug = false; //have a verbose serial debug option
bool enableDrive = true;  // be able to turn off motors for sensor debug

void setup()
{
  Serial.begin(9600);

  initalizeMotoShieldPins(); // setup H-bridge pins
  pinMode(sonicSensor, INPUT); // Initalize sonic sensor pin
  currentTime = millis(); //Start keeping track of time


}


void loop() {
  currentTime = millis(); //update P-on time, should be first action of every loop
  currentHeading = getCurrentHeading(); // update our current heading
  updateDesiredHeading(); //update our desired heading


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
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}


////////////////Function to start motors ////////////////
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
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

//////////////////starts motors while checking for obstacles////////////////

/*
  void motorStart() {
  //starts motor
  if (obstacleCheck(distance) >= obstacleDistance) {
    motorGo(0, CW, 1023);
    motorGo(1, CW, 1023);
  }
  else {
    motorGo(0, CW, 0);
    motorGo(1, CW, 0);

    void turn();
  }

  }
*/

/*********** Check for shit in our way, return true if found ********************/
bool  obstacleCheck() {
  int sensorRead = analogRead(sonicSensor); //sets local var to ADC value
  
  return false; // PLACEHOLDER - TO BE REMOVED

}


////////////////turn to avoid obstacle////////////////
void obstacleTurn() {
  //change heading ex currentheading - 30

  // determine direction of heading that is closer to beacon

  // reverse 2 feet

  // turn towards beacon 70 deg

  //check for obstacles, if found turn 70 deg in other direction and recheck

  //drive forward for ~6 feet in while loop whilst checking for obstacles (A great oppertunity for recursiuon!)

  //
  delay(1000);
  // find beacon heading
  // break; Only used in loops or switch
}

//********* turn towards beacon when called **********
void beaconTurn(int heading) { //heading is diseried direction

}

/********* Returns Robot's current Heading *********/
int getCurrentHeading() {
  int heading = 0; // PLACEHOLDER

  //TODO: set heading to compass heading read

  return heading;

}
/********* Gets desired robot heading **********/
void updateDesiredHeading() {

}

