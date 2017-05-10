#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <U8g2lib.h>

// If using software SPI (the default case):
#define OLED_CLK   12   //D0 SCL,CLK,SCK Clock
#define OLED_MOSI  11   //D1 SDA MOSI Data
#define OLED_RESET 10   //RES RST RESET Reset
#define OLED_DC    9    //A0 Data/Command
#define OLED_CS    8    //CS Chip Select

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
U8G2_SSD1306_128X64_NONAME_2_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ OLED_CLK, /* data=*/ OLED_MOSI, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);

#define LED_PIN 13 //

//#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 1

// Singleton instance of the radio driver
RH_ASK driver(2000,2); //,NULL,NULL);
// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(driver, SERVER_ADDRESS);

struct dataStruct{
int roll = 0;    //Roll
int pitch = 0;   //Pitch
int heading = 0; //Magnetic bearing
  }SensorReadings;

void setup() 
{
  u8g2.begin();
}

void loop()
{  
  ReciveFromSensors();
  draw();
}

void ReciveFromSensors()
{
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

void draw()
{
  u8g2.firstPage();
  do {
    //u8g2.setFont(u8g2_font_ncenB14_tf ); //14 pixel high
    //u8g2_font_profont22_mf 14 pixel monospace
    // u8g2_font_helvB12_tf 12 pixel Helvetica
    // u8g2_font_crox3c_mf 12 pixel monospace
    u8g2.setFont(u8g2_font_helvB12_tf );
    u8g2.drawStr(0,12,"Hello World!");
    char text[13];
    sprintf(text, "Pitch: %i", (int)SensorReadings.pitch);
    u8g2.drawStr(0,30,text);
    sprintf(text, "Roll: %i", (int)SensorReadings.roll);
    u8g2.drawStr(0,45,text);
    //u8g2.drawStr(0,12,"Hello World!");
  } while ( u8g2.nextPage() );
}
