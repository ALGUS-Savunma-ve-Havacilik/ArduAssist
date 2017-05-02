#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*
  U8glib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
*/

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
//U8G2_SSD1306_128X64_NONAME_2_4W_SW_SPI u8g2(U8G2_MIRROR, /* clock=*/ OLED_CLK, /* data=*/ OLED_MOSI, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);

typedef u8g2_uint_t u8g_uint_t;

#define SECONDS 10
uint8_t flip_color = 0;
uint8_t draw_color = 1;

void draw_set_screen(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g2.setColorIndex(flip_color);
  u8g2.drawBox( 0, 0, u8g2.getWidth(), u8g2.getHeight() );
}
void setup() {
  // put your setup code here, to run once:
    u8g2.begin();
  // flip screen, if required
  // u8g2.setRot180();
  
  // assign default color value
  //draw_color = 1;         // pixel on

}

void loop() {
  // put your main code here, to run repeatedly:
    u8g2.firstPage();
  do {
    //u8g2.setFont(u8g2_font_ncenB14_tf ); //14 pixel high
    //u8g2_font_profont22_mf 14 pixel monospace
    // u8g2_font_helvB12_tf 12 pixel Helvetica
    // u8g2_font_crox3c_mf 12 pixel monospace
    u8g2.setFont(u8g2_font_helvB12_tf );
    u8g2.drawStr(0,12,"Hello World!");
  } while ( u8g2.nextPage() );
  delay(1000);

}
