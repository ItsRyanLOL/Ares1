//Beacon heading does not need to be printed, robot heading may need to be printed

#include <XBee.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial outputSerial(10, 9); // RX, TX

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
int resetRSSI = -1000;    //The value that RSSI is reset to after each pass through filter
#define samples 110
short beaconHeading;

void setup() {
  //Initialize serial communications at 57600 bps:
  Serial.begin(57600); 
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
  outputSerial.begin(57600);

  //Initialize i2c communications
  Wire.begin(8);                // join i2c bus with address #8
  //Wire.onRequest(i2cPrint); // register event
}

void loop{  
  //Process the data and reset.
  bHeading  = (ProcessData());
  beacHeadingI2c[0] = 0xFF&(bHeading >>8);
  beacHeadingI2c[1] = 0xFF&bHeading ;
  Serial.println(bHeading );
  outputSerial.println(bHeading );
}

//retrieves beacon info
void Retrieve(int i){
  xbee.readPacket(10);    //Wait 50 to receive packet
  if (xbee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) //****What is getApiId()?
    {
      xbee.getResponse().getRx16Response(rx16);
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++) headingConverter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();

      //Write to array
      readings[i].bHeading = headingConverter.f;
      readings[i].signalStrength = currentRSSI;
    }
  }else{
    readings[i].beaconbHeading = 0;
    readings[i].signalStrength = resetRSSI;
  }
}

//Creates a bHeading  through averaging the readings
int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;
  maxRSSI = readings[0].signalStrength;
  
  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }
  //If there is no valid data
  if(maxRSSI == resetRSSI){
    return -1;
  }

  float headingx = 0;
  float headingy = 0;
  for(int i = 0; i < samples; i++)
  {
    if (readings[i].signalStrength == -1000 && readings[i].bHeading  == 0)
    {
       Serial.println("this bHeading  not included");
    }
    else
    {
      Serial.print(readings[i].heading);
      Serial.print("\t");
      Serial.println(readings[i].signalStrength);
      // Set magnitude of vector by signal strength
      headingx += readings[i].signalStrength * cos(readings[i].bHeading  * PI / 180);
      headingy += readings[i].signalStrength * sin(readings[i].bHeading  * PI / 180);
    }
  }
  
  float bHeading  = atan2(BeacYHeading, BeacXheading);
  if (bHeading  < 0) bHeading  += 2 * PI;
  bHeading  = bHeading  * 180 / PI;

  return (int) heading;
}
