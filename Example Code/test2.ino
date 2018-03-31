/*Code to update time, currentHeading, desiredHeading, check distance from beacon, and check for obstacles
 * 
 */
 int sigStrength;
 int beacSigStrength;
 
 void setup() {
  // put your setup code here, to run once:

}

void loop() {
  //update time:  currentTime = millis();
  currentTime = millis();
  
  //update currentHeading: returns a variable
  AresHeading();

  //update desiredHeading: return a variable
  DesiredHeading();

  //check if robot is close to desired heading: boolean(true = drive, false = turn)
  CheckHeadingDifference();
  //check for obstacles: if(currentTime >= (checkInterval + lastCheck)
    // true = avoids, false = drive
  

}

int AresHeading(){
  //find compass heading
  return currentHeading;
}

int DesiredHeading(){
  //find beacon heading
  //desiredHeading = beaconHeading - 180;
  return desiredHeading;
}

bool CheckHeadingDifference(){
  if((currentHeading >= desiredHeading - 5) && (currentHeading <= desiredHeading + 5))
    return true;
  else
    return false;

}

bool obstacleCheck(){
  int sensorRead = analogRead(sonicSensor); //sets local var to ADC value
  
  return false; // PLACEHOLDER - TO BE REMOVED
}

