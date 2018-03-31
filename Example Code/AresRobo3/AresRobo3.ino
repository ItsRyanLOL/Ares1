#define BRAKEVCC 0
#define CW   1 //clockwise
#define CCW  2 //counterclockwise
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)
int interval = 2500; //interval at which to check surroundings

int statpin = 13;

//sets obstance distance to avoid at 2 ft
int obstacleDistance = 24;

void setup()
{
  Serial.begin(9600);

  void initalizeMotoShieldPins();
  
}


void loop() {
  
}




////////////////Initalize MotoShield Pins ////////////////
initalizeMotoShieldPins(){
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
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
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

//////////////////starts motors while checking for obstacles////////////////
void motorStart(){
  //starts motor
  if (obstacleCheck(distance) >= obstacleDistance) {
    motorGo(0, CW, 1023);
    motorGo(1, CW, 1023);
  }
  else{
      motorGo(0,CW,0);
      motorGo(1,CW,0);

      void turn();
      }
  
}

////////////////turn to avoid obstacle////////////////
void turn() {
  //change heading ex currentheading - 30
  delay(1000);
  // find beacon heading
  break;
  
}

