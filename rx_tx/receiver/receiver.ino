#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h>

//#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 1

// Singleton instance of the radio driver
RH_ASK driver(2000,2,NULL,NULL);
// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(driver, SERVER_ADDRESS);

void setup() 
{
}

void loop()
{  
ReciveFromSensors();
}

void ReciveFromSensors()
{

  struct dataStruct{
  int roll = 0;    //Roll
  int pitch = 0;   //Pitch
  int heading = 0; //Magnetic bearing
}SensorReadings;

  // Dont put this on the stack:
  uint8_t buf[sizeof(SensorReadings)];
  uint8_t from;
  uint8_t len = sizeof(buf);
  
  if (manager.available())
  {    
    // Wait for a message addressed to us from the client
    if (manager.recvfrom(buf, &len, &from))
    {
      memcpy(&SensorReadings, buf, sizeof(SensorReadings));
       
      Serial.print(F("YPR: "));
      Serial.print(SensorReadings.heading);
      Serial.print(F(" "));
      Serial.print(SensorReadings.pitch);
      Serial.print(F(" "));
      Serial.print(SensorReadings.roll);
    }
  }
}
