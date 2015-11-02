#include "Arduino.h"
//#include "HardwareSerial.h" 
#include "lcd_util.h"
//#include "touch.h"
//#include "magnometer.h"
//#include "sdcard.h"
//#include "temperature.h"


#include <UTouch.h>
#include <UTFT_DLB_Buttons.h>
#include <UTFT.h>
#include <UTFT_DLB.h>

extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t SevenSegNumFont[];
extern uint8_t DejaVuSans18[];
extern uint8_t DejaVuSans24[];

extern UTFT_DLB myGLCD;
extern UTouch myTouch;
extern UTFT_DLB_Buttons myButtons;

//**********************************************

void waitForTouch()
{
  while (myTouch.dataAvailable() == true) {}
  while (myTouch.dataAvailable() == false) {}
  while (myTouch.dataAvailable() == true) {}
}
//*********************************************

//***********************************************
void drawCrossHair(int x, int y)
{
  myGLCD.drawRect(x-10, y-10, x+10, y+10);
  myGLCD.drawLine(x-5, y, x+5, y);
  myGLCD.drawLine(x, y-5, x, y+5);
}
//***********************************************

//***********************************************
void drawMenu_main()
{
    Serial.println("drawMenu_main");
//    myGLCD.clrScr();
    myGLCD.drawRect(5,5,799,469);
    myGLCD.drawRect(140,384,668,459);
}
//**********************************************

//**************************************************************************************************************
void drawBttn_main()
{
  myButtons.setButtonColors(0, VGA_WHITE, VGA_GRAY, VGA_RED, VGA_WHITE, VGA_BLUE); // individual button colors
  myButtons.drawButton(0);
  myButtons.setButtonColors(1, VGA_WHITE, VGA_GRAY, 0x07E4, VGA_WHITE, VGA_MAROON); // individual button colors
  myButtons.drawButton(1);
  myButtons.setButtonColors(2, VGA_WHITE, VGA_GRAY, 0x07E4, VGA_WHITE, VGA_FUCHSIA); // individual button colors
  myButtons.drawButton(2);
  myButtons.setButtonColors(3, VGA_BLACK, VGA_GRAY, 0x07E4, VGA_WHITE, VGA_AQUA); // individual button colors
  myButtons.drawButton(3);
  myButtons.setButtonColors(4, VGA_BLACK, VGA_GRAY, VGA_BLUE, VGA_WHITE, VGA_RED); // individual button colors
  myButtons.drawButton(4);
  myButtons.setButtonColors(5, VGA_BLACK, VGA_GRAY, 0x07E4, VGA_WHITE, VGA_SILVER); // individual button colors
  myButtons.drawButton(5);
  myButtons.setButtonColors(6, VGA_BLACK, VGA_GRAY, 0x07E4, VGA_WHITE, VGA_YELLOW); // individual button colors
  myButtons.drawButton(6);
  myButtons.setButtonColors(7, VGA_BLACK, VGA_GRAY, VGA_WHITE, VGA_BLUE, VGA_GREEN); // individual button colors
  myButtons.drawButton(7);  
  
}
//******************************************************************************************************************

//*********************************************************
// Draw MENU and PAGE BUTTONS
//*********************************************************
void drawMenu_sub()
{
   myGLCD.InitLCD();
   myGLCD.clrScr();
   myGLCD.drawRect(10,10,799,469);
   myGLCD.drawRect(140,384,668,459);
   myGLCD.setFont(DejaVuSans18);
   myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_BLUE);
   myButtons.drawButton(6);
   myButtons.setButtonColors(7, VGA_BLACK, VGA_GRAY, 0xF800, VGA_WHITE, 0xF800);  // individual button colors
   myButtons.drawButton(7);

  
}
