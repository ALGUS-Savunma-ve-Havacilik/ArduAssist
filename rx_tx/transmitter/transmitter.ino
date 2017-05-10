#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h>

//RH communication
#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1
// Singleton instance of the radio driver
RH_ASK driver;
// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(driver, CLIENT_ADDRESS);

  struct dataStruct{
  int roll = 0;    //Roll
  int pitch = 0;   //Pitch
  int heading = 0; //Magnetic bearing
  }SensorReadings;

 // RF communication, Dont put this on the stack:
  byte buf[sizeof(SensorReadings)] = {0};

void setup() 
{
  Serial.begin(9600);
  
  //RF communication
  if (!manager.init())
    Serial.println("init failed");
}

void loop()
{  
  SendValues(); //Send sensor values
}

//RF communication
void SendValues()
{    
  //Load message into data-array
  //byte data[sizeof(sensorValues)];

  byte zize=sizeof(SensorReadings);
  memcpy (buf, &SensorReadings, zize);
    
  // Send a message to manager_server
  while (!manager.sendto(buf, zize, SERVER_ADDRESS))
    {
  }
}
