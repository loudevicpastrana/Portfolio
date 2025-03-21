/*
 * Loudevic Pastrana
 * 
 *
 * (c) 2021 Cristian Pop
 */

#include "test_all.h"
#include <stdio.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>

#include "mp3_control.h"


#define TS_MINX 1  // Minimum X value (calibrate for your screen)
#define TS_MINY 8   // Minimum Y value (calibrate for your screen)
#define TS_MAXX 237   // Maximum X value (calibrate for your screen)
#define TS_MAXY 315   // Maximum Y value (calibrate for your screen)

#define MINPRESSURE 1  // Minimum valid pressure for a touch
#define MAXPRESSURE 200 // Maximum valid pressure for a touch

static long MapTouchToScreen(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Adafruit_ILI9341 lcdCtrl = Adafruit_ILI9341(); // The LCD controller
Adafruit_FT6206 touchCtrl = Adafruit_FT6206(); // The Touchscreen controller

// Add a button instance here (similar to lcdCtrl and touchCtrl above): 
// look at Adafruit_GFX.h, the Adafruit_GFX_Button class.


Adafruit_GFX_Button myButton_play;
Adafruit_GFX_Button myButton_stop;
Adafruit_GFX_Button myButton_next;

// Define constants for button position and size
#define BUTTON_X 50
#define BUTTON_Y 280
#define BUTTON_WIDTH 50
#define BUTTON_HEIGHT 50

#define BUTTON_STOP_X 120
#define BUTTON_STOP_Y 280
#define BUTTON_STOP_WIDTH 50
#define BUTTON_STOP_HEIGHT 50

#define BUTTON_NEXT_X 200
#define BUTTON_NEXT_Y 280
#define BUTTON_NEXT_WIDTH 50
#define BUTTON_NEXT_HEIGHT 50

/************************************************************************************

   Runs LCD/Touch GUI task

************************************************************************************/
// Define a function to initialize the LCD and touchscreen controllers
static void initControllers() {
  SPI1Lock();
  lcdCtrl.begin();
  touchCtrl.begin(40);
  SPI1Unlock();
}

// // Define a function to draw the button on the screen
static void drawButton() {
  myButton_play.initButton(&lcdCtrl, BUTTON_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT, 
    ILI9341_WHITE, ILI9341_BLUE, ILI9341_WHITE, "Play", 2);
myButton_play.drawButton();

myButton_stop.initButton(&lcdCtrl, BUTTON_STOP_X, BUTTON_STOP_Y, BUTTON_WIDTH, BUTTON_HEIGHT, 
    ILI9341_WHITE, ILI9341_RED, ILI9341_WHITE, "Stop", 2);
myButton_stop.drawButton();

myButton_next.initButton(&lcdCtrl, BUTTON_NEXT_X, BUTTON_NEXT_Y, BUTTON_WIDTH, BUTTON_HEIGHT, 
  ILI9341_WHITE, ILI9341_GREEN, ILI9341_WHITE, "Next", 2);
myButton_next.drawButton();
}

 static void drawGui()
 {
   lcdCtrl.fillScreen(lcdCtrl.color565(50/3, 0, 111/3));

   lcdCtrl.setCursor(0, ILI9341_TFTHEIGHT / 2 - 100);
   lcdCtrl.setTextColor(ILI9341_WHITE);
   lcdCtrl.setTextSize(2);
   char printBuffer[128];
   lcdCtrl.print(printBuffer, sizeof(printBuffer), "PLAYING:");
   lcdCtrl.print(printBuffer, sizeof(printBuffer), mp3Files[currentSongIndex]);
   lcdCtrl.setCursor(100, ILI9341_TFTHEIGHT / 2 - 70);

   drawButton(); 

}




// Define a function to handle touch events
static void handleTouchEvent() {

  static Mp3Command_t cmd;
  //OS_ERR err;

  if (!touchCtrl.touched()) return;

  TS_Point point = touchCtrl.getPoint(); 

  if (point.x == 0 && point.y == 0) return;  // Ignore spurious touches

  TS_Point p;
  p.x = MapTouchToScreen(point.x, TS_MAXX, TS_MINX, 0, ILI9341_TFTWIDTH);  
  p.y = MapTouchToScreen(point.y, TS_MAXY, TS_MINY, 0, ILI9341_TFTHEIGHT);  

  // Play Button
  if (myButton_play.contains(p.x, p.y)) {
    myButton_play.press(true);
    myButton_play.drawButton(true);
    OSTimeDly(50);
    cmd = MP3_CMD_PLAY;
    OSQPost(mp3CmdQueue, (void *)&cmd);
    myButton_play.press(false);
    myButton_play.drawButton(false);
    drawGui();

  }

  // Stop Button
  if (myButton_stop.contains(p.x, p.y)) {
    myButton_stop.press(true);
    myButton_stop.drawButton(true);
    OSTimeDly(50);
    cmd = MP3_CMD_STOP;
    OSQPost(mp3CmdQueue, (void *)&cmd);
    myButton_stop.press(false);
    myButton_stop.drawButton(false);

    drawGui();
  }

  // Stop Button
  if (myButton_next.contains(p.x, p.y)) {
    myButton_next.press(true);
    myButton_next.drawButton(true);
    OSTimeDly(50);
    cmd = MP3_CMD_NEXT;
    OSQPost(mp3CmdQueue, (void *)&cmd);
    myButton_next.press(false);
    myButton_next.drawButton(false);

    drawGui();
  }


 
}


// Define the GuiTask function
void GuiTask(void* pdata) {
  log_debug("GuiTask: starting\n");

  initControllers();

  drawButton();
  
  while (1) {
      handleTouchEvent();
      OSTimeDly(50);
  }
}