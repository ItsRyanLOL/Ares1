//RIGHT AND LEFT NEED TO BE OPPOSITE
//set up clock instead of delay

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

void setup()
{
  Serial.begin(9600);
  
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
  // motorGo(0, CW, 1023); //1023 is HIGH
  // motorGo(1, CCW, 1023);
}

//if want 50% power go from 1023 to 1023/2
void loop() {
  //motor goes forward, checks every 2.5 seconds
  motorGo(0, CW, 1023);
  motorGo(1, CW, 1023);
  /*
   time = millis() + oneDelay;
   */
  delay(2500);
  

}

//sets up motor to go
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


