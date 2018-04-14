// Beacon Receiver with Task Scheduling for use in main processor

#include <Wire.h> // I2C Libary
const int xbeeWireAddress = 8; //I2C adress of slave xbee


#include "stdlib.h"
#include "TaskScheduler.h"
#include <XBee.h>

#define XBeeNavSerial Serial1  // Change to user desired port, the one wired to the XBee

#define NAVSAMPLES 250         // Samples of transmitted navigation beacon packets, needs to be larger than packets per revolution

#define RXXBEENAV_RATE 8       // Rate to look for a beacon packet in msec, select a rate that is slightly faster than the packets are sent
#define NAV_RATE 500           // Rate to calculate beacon radial, select a rate that is near one revolution of the beacon

#define FALSE 0
#define TRUE 1

//Structure to contain the readings from the beacon
struct
{
  float Radial;  // -180 to 180 degrees
  int RSSI;
} NavReadings[NAVSAMPLES];

// Union for converting between byte[4] and float
union
{
  float f;
  uint8_t b[4];
} RadialConverter;

int i, j, BaudRate, StopIndex, BeforeI, OneRevIndex, XBeeNavCount, CurrentRSSI, NavReadingsIndex = 0, NavReadingsIndexLast = 0;
int MinRSSI, MaxRSSI, RSSIRange, ThreshRSSI, BeaconRSSI;
float CompX, CompY, BeaconX, BeaconY, BeaconRad, BeaconBrg, CurrentRadialReading;
boolean NavReadingsRolled = FALSE, OneRev;

XBee xbeeNav = XBee();  // This instantiates the XBee library for the XBee that listens to the Navigation Beacon

XBeeResponse response = XBeeResponse();
Rx16Response rx16Nav = Rx16Response();  // Create object to contain the Navigation packets

void RxXBeeNavCallBack();  // The task that gets Nav Beacon packets from the XBee
void NavCallBack();        // The task that calculates the beacon radial

Task tRxXBeeNav(RXXBEENAV_RATE,TASK_FOREVER,&RxXBeeNavCallBack);
Task tNav(NAV_RATE,TASK_FOREVER,&NavCallBack);

Scheduler taskManager;

void RxXBeeNavCallBack()
{
//  Attempt to read an API packet from the XBee Module
  xbeeNav.readPacket();
    
//  If we found a packet...
  if (xbeeNav.getResponse().isAvailable())
  {
  //  Got something
  //  If it's a rx16 Receive packet (API ID 0x81)...
    if (xbeeNav.getResponse().getApiId() == RX_16_RESPONSE)
    {
    //  Got a rx packet
    //  Populate our "rx" object with info from the received packet (Data, Addresses, etc)
      xbeeNav.getResponse().getRx16Response(rx16Nav);
    //  Store the transmitted data and RSSI
      for(j = 0; j < 4; j++) RadialConverter.b[j] = rx16Nav.getData(j);
      CurrentRSSI = -rx16Nav.getRssi();  // XBee returns positive value so Current RSSI is always negative
    //  Write to array
      NavReadings[NavReadingsIndex].Radial = RadialConverter.f;
      // Convert from 0 to 360 to -180 to 180 range
      if(NavReadings[NavReadingsIndex].Radial >= 180) NavReadings[NavReadingsIndex].Radial -= 360;
      NavReadings[NavReadingsIndex].RSSI = CurrentRSSI;
//      Serial.print("Nav Readings Index:  ");
      Serial.print(NavReadingsIndex);  Serial.print(",");
//      Serial.print("  Millis:  ");
      Serial.print(millis());  Serial.print(",");
//      Serial.print("  Radial:  ");
      Serial.print(NavReadings[NavReadingsIndex].Radial);  Serial.print(",");
//      Serial.print("  RSSI:  ");
      Serial.println(NavReadings[NavReadingsIndex].RSSI);          
      NavReadingsIndex++;
      if(NavReadingsIndex >= NAVSAMPLES) 
      { 
        NavReadingsIndex = 0;
        NavReadingsRolled = TRUE;
//        Serial.print("Nav Readings Rolled Over at millis: "); Serial.println(millis());
      }
    }
  }  // if isAvailable
}  // CallBack

void NavCallBack()
{
//  Calculate Beacon Radial from the readings stored in the RxXBeeNav task
//  Go back through the ring buffer for one revolution of the beacon
//  First find the index number for the beginning of the revolution and find max and min RSSI
//  Then apply a circular filter to the readings that are above an RSSI threshold to determine the radial from the station
  if (NavReadings > 0) i = NavReadingsIndex - 1; else i = NAVSAMPLES - 1;  // Was incremented at the end of the read
  OneRev = FALSE;  CurrentRadialReading = NavReadings[i].Radial;
  MaxRSSI = -100;  MinRSSI = 0;      // The NavReadings stores RSSI as its actual negative dBm number
  if (NavReadingsRolled) StopIndex = i + 1; else StopIndex = 0;  // Stop Index is how far back the ring buffer readings have been populated
  if (StopIndex == NAVSAMPLES) StopIndex = 0;
  while(i != StopIndex && !OneRev)
  {
    if(MaxRSSI < NavReadings[i].RSSI) MaxRSSI = NavReadings[i].RSSI;
    if(MinRSSI > NavReadings[i].RSSI) MinRSSI = NavReadings[i].RSSI;
    if(i > 0) BeforeI = i - 1; else BeforeI = NAVSAMPLES - 1;
    // Look for the previous reading that is the closest angle to the most recent reading to find one revolution
    if(NavReadings[i].Radial > CurrentRadialReading && NavReadings[BeforeI].Radial < CurrentRadialReading) OneRev = TRUE;
    if((NavReadings[i].Radial - CurrentRadialReading) < (CurrentRadialReading - NavReadings[BeforeI].Radial))OneRevIndex = i; else OneRevIndex = BeforeI;
    i--; if(i < 0) i = NAVSAMPLES - 1;
  }
  // Apply a circular filter to the readings from the last revolution that have RSSI above the threshold RSSI
  // Weight them by the square of the distance the RSSI is above the thresshold RSSI
  i = NavReadingsIndex - 1;  BeaconX = BeaconY = 0.0;
  ThreshRSSI = MaxRSSI - 7;  // Only readings with RSSI greater than the threshold will be used, others may be reflections
  RSSIRange = MaxRSSI - ThreshRSSI;
  if(OneRevIndex > 0) OneRevIndex = OneRevIndex - 1; else OneRevIndex = NAVSAMPLES - 1;
  while(i != OneRevIndex)
  {
//    Serial.print("Nav Readings Index:  "); Serial.print(i);
//    Serial.print(",  Radial:  "); Serial.print(NavReadings[i].Radial);
//    Serial.print(",  RSSI:  "); Serial.println(NavReadings[i].RSSI);    
    if((MaxRSSI - NavReadings[i].RSSI) < RSSIRange)
    {
      CompX = pow((NavReadings[i].RSSI - ThreshRSSI),2) * cos(NavReadings[i].Radial * PI / 180);
      CompY = pow((NavReadings[i].RSSI - ThreshRSSI),2) * sin(NavReadings[i].Radial * PI / 180);
      BeaconX = BeaconX + CompX;
      BeaconY = BeaconY + CompY;
      Serial.print(",,,,");
//      Serial.print("In Threshold Nav Readings Index:  ");
      Serial.print(i);  Serial.print(",");
//      Serial.print("  Radial:  "); 
      Serial.print(NavReadings[i].Radial);  Serial.print(",");
//      Serial.print("  RSSI:  ");
      Serial.print(NavReadings[i].RSSI);  Serial.print(",");   
//      Serial.print("  Weight:  ");
      Serial.print(pow((NavReadings[i].RSSI - ThreshRSSI),2));  Serial.print(",");
//      Serial.print("  Beacon X:  ");
      Serial.print(CompX);  Serial.print(",");     
//      Serial.print("  Beacon Y:  ");
      Serial.println(CompY);
    }
    i--; if(i < 0) i = NAVSAMPLES - 1;  
  }
  BeaconRad = (atan2(BeaconY,BeaconX) * 180 / PI);
  BeaconBrg = (BeaconRad + 180);  // Bearing is to the station and is radial + 180
  if(BeaconBrg > 180) BeaconBrg -= 360;
  BeaconRSSI = MaxRSSI;
  Serial.print(",,,,,,,,,,");
//  Serial.print("Beacon Radial Calculation at Millis:  ");
//  Serial.print(millis());  Serial.print(",");
//  Serial.print("  Nav Readings Index:  ");
//  Serial.print((NavReadingsIndex > 0) ? NavReadingsIndex - 1 : NAVSAMPLES - 1);  Serial.print(",");
//  Serial.print("  Stop Index:  ");
//  Serial.print(StopIndex);  Serial.print(",");
  Serial.print("  Current Radial Reading:  ");
  Serial.print(CurrentRadialReading);  Serial.print(",");
  Serial.print("  One Rev Index:  ");
  Serial.print((OneRevIndex < NAVSAMPLES - 1) ? OneRevIndex + 1 : 0);  Serial.print(",");
    Serial.print("  Beacon Radial:  ");
  Serial.print(BeaconRad);  Serial.print(",");
  Serial.print("  Beacon Bearing:  ");
  Serial.print(BeaconBrg);  Serial.print(",");
  Serial.print("  Min RSSI:  ");
  Serial.print(MinRSSI);  Serial.print(",");
  Serial.print("  Max RSSI:  ");
  Serial.println(MaxRSSI);  
}

void setup()
{
  Serial.begin(115200);  // To talk to PC Arduino Monitor Screen or better yet RealTerm for data capture

 //  Set Arduino xbeeNav serial port to 57,600 baud
  BaudRate = 57600;
  XBeeNavSerial.begin(BaudRate);
  xbeeNav.setSerial(XBeeNavSerial);  // Tell XBeeNav which serial port to use, use XCTU tool to set the XBee baud rate

  taskManager.init();
  taskManager.addTask(tRxXBeeNav);
  taskManager.addTask(tNav);  

  tRxXBeeNav.enable();  //  This starts the tasks and should be the last line of the setup section
  tNav.enable();
}

void loop()
{
  taskManager.execute();
}
