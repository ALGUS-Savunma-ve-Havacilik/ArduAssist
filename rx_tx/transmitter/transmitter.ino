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
  float tempValue;
  int soilValue;
  int soilRaw;
  unsigned long counter;
   
}SensorReadings;

 // RF communication, Dont put this on the stack:
  byte buf[sizeof(SensorReadings)] = {0};

void setup() 
{
  Serial.begin(9600);
  
  //RF communication
  if (!manager.init())
    Serial.println("init failed");

  SensorReadings.tempValue = 0;
  SensorReadings.soilValue = 0;
  SensorReadings.soilRaw = 0;
  SensorReadings.counter = 0;
}

void loop()
{
  Serial.println("------------------------------------");
  float tempValue;
  int soilValue;
  
  temperature(); //Read temp sensor
  delay(100);
  
  soilValue = soil(); //Read hygrometer
  delay(100);
  
  SendValues(); //Send sensor values
  delay(200);
  Serial.println("------------------------------------");
}

//Get temperatures from Dallas sensor
void temperature()
{ 
  // issue a global temperature request to all devices on the bus
  
  SensorReadings.tempValue = millis() * .01;
  Serial.println(SensorReadings.tempValue);
}

//Soil hygrometer
int soil()
{
 // read soil moisture from sensor

 SensorReadings.soilRaw = (int)(millis() *.014);
 Serial.println(SensorReadings.soilRaw);
}

//RF communication
void SendValues()
{  
  //Load message into data-array
  //byte data[sizeof(sensorValues)];

  byte zize=sizeof(SensorReadings);
  memcpy (buf, &SensorReadings, zize);

  
  Serial.println("Sending to ask_datagram_server");
  int i = 0;
  int repeat = 1; //Times to repeat same message
    
  // Send a message to manager_server
  while (i<repeat)
  {
    if (manager.sendto(buf, zize, SERVER_ADDRESS))
    {
      Serial.println("Message sent");   
    }
    else
    {
      Serial.println("sendto failed");
    }
    delay(100);
    i = i+1;
  }
  SensorReadings.counter = SensorReadings.counter + 1;
}
