#include "Arduino.h"

//
//#include <UTouch.h>
//#include <UTFT_Buttons.h>
//#include <UTFT.h>

//extern UTFT myGLCD;
//extern UTouch myTouch;
//extern UTFT_DLB_Buttons myButtons;




#ifndef HEADER_LCDUTIL
  #define HEADER_LCDUTIL
  void drawCrossHair(int x, int y); 
    #endif
    
  #ifndef HEADER_WAITFORTOUCH
  #define HEADER_WAITFORTOUCH
    void waitForTouch();
  #endif
  
  #ifndef HEADER_DRAWMENUMAIN
  #define HEADER_WDRAWMENUMAIN
    void drawMenu_main();
  #endif
  
//  #ifndef HEADER_DRAWBTTNMAIN
//  #define HEADER_WDRAWBTTNMAIN
//    void drawBttn_main();
//  #endif
  
  #ifndef HEADER_DRAWMENUSUB
  #define HEADER_WDRAWMENUSUB
    void drawMenu_sub();
  #endif
  
    #ifndef HEADER_DRAWMENUSUB
  #define HEADER_WDRAWMENUSUB
    void drawBttn_main();
  #endif
  
