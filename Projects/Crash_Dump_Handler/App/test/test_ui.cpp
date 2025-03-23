/*
 * University of Washington
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>

#include "main.h"

#include "Adafruit_ILI9341.h"
#include "Adafruit_FT6206.h"

#define TS_MINX 150   // Minimum X value (calibrate for your screen)
#define TS_MINY 120   // Minimum Y value (calibrate for your screen)
#define TS_MAXX 920   // Maximum X value (calibrate for your screen)
#define TS_MAXY 900   // Maximum Y value (calibrate for your screen)

#define MINPRESSURE 1  // Minimum valid pressure for a touch
#define MAXPRESSURE 200 // Maximum valid pressure for a touch



static long MapTouchToScreen(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t test_ui()
{
  int32_t ret = 0;

  printf("-- TEST: UX\n");

  Adafruit_ILI9341 lcdCtrl = Adafruit_ILI9341(); // The LCD controller
  Adafruit_FT6206 touchCtrl = Adafruit_FT6206(); // The Touchscreen controller

  // UW_TODO: Add a button instance here (similar to lcdCtrl and touchCtrl above): 
  //          look at Adafruit_GFX.h, the Adafruit_GFX_Button class.


  Adafruit_GFX_Button myButton;


  lcdCtrl.begin();

  lcdCtrl.fillScreen(lcdCtrl.color565(50/3, 0, 111/3));

  lcdCtrl.setCursor(0, ILI9341_TFTHEIGHT / 2 - 100);
  lcdCtrl.setTextColor(ILI9341_WHITE);
  lcdCtrl.setTextSize(2);
  char printBuffer[128];
  lcdCtrl.print(printBuffer, sizeof(printBuffer), "University of ");
  lcdCtrl.setCursor(100, ILI9341_TFTHEIGHT / 2 - 70);
  lcdCtrl.print(printBuffer, sizeof(printBuffer), "Washington");

  lcdCtrl.setTextSize(1);
  lcdCtrl.setTextColor(ILI9341_YELLOW);
  lcdCtrl.setCursor(0, ILI9341_TFTHEIGHT - 10);
  lcdCtrl.print(printBuffer, sizeof(printBuffer), "Touch the screen to draw.");

  touchCtrl.begin(40);

  // UW_TODO: Initialize the button here choosing position, color, label, etc. The button will be 
  //          attached to lcdCtrl which inherits from Adafruit_GFX:
  // button.initButton(&lcdCtrl, ...)
  // Hint: use ILI9341_TFTHEIGHT and ILI9341_TFTWIDTH for positioning.
  // Hint: use any of the ways of specifying a color used above.
  
  // UW_TODO: Draw the button.

  myButton.initButton(&lcdCtrl, 105, 150, 200, 50, ILI9341_WHITE, ILI9341_BLUE, ILI9341_WHITE, "Press Me", 2);
  myButton.drawButton();  // Draw the button on the display
  
  while(1)
  {
    bool touched = touchCtrl.touched();
           if (!touched)
    {
      continue;
    }

    TS_Point point = touchCtrl.getPoint();
    if (point.x == 0 && point.y == 0)
    {
        continue; // usually spurious, so ignore
    }
    
    // transform touch orientation to screen orientation.
    TS_Point p = TS_Point();
    p.x = MapTouchToScreen(point.x, 0, ILI9341_TFTWIDTH, ILI9341_TFTWIDTH, 0);
    p.y = MapTouchToScreen(point.y, 0, ILI9341_TFTHEIGHT, ILI9341_TFTHEIGHT, 0);
    
    lcdCtrl.fillCircle(p.x, p.y, 3, ILI9341_RED);

    // UW_TODO: Check if the button was pressed by checking if the button contains the mapped 
    //          coordinates. Hint: you can draw it inverted to show it was pressed and delay 500ms 
    //          to debounce (avoid registering too many presses).

   bool pressed = touchCtrl.touched();

    if (pressed == true) // Check for valid touch
    {  
      // Map the touch coordinates to display coordinates
      pressed = 0;
      TS_Point point = touchCtrl.getPoint();

      int x = point.x;
      int y = point.y;

      if (point.z >= MINPRESSURE && point.z < MAXPRESSURE) 
      {

        //point.x = MapTouchToScreen(point.x, TS_MINX, TS_MAXX, 0, lcdCtrl.width());
       // point.y = MapTouchToScreen(point.y, TS_MINY, TS_MAXY, 0, lcdCtrl.height());

        // Check if the touch is within the button area
        if (myButton.contains(x, y)) 
        {
          myButton.press(true);  // Button is pressed
          myButton.drawButton(true);  // Redraw the button in the pressed state
            // Set screen rotation to 180 degrees (flips everything upside down)
          HAL_Delay(500);  // 500 ms delay
        // Clear the display
         lcdCtrl.fillScreen(ILI9341_BLACK);
         lcdCtrl.setTextColor(ILI9341_WHITE);
         lcdCtrl.setTextSize(2);
         lcdCtrl.setCursor(0, ILI9341_TFTHEIGHT / 2 - 100);
         lcdCtrl.print(printBuffer, sizeof(printBuffer), "Button Pressed");

          HAL_Delay(100);  // 100 ms delay

          lcdCtrl.setRotation(2);  // Set rotation (0, 1, 2, or 3)

          HAL_Delay(500);  // 500 ms delay

          // After delay, redraw the button in its normal state
          myButton.press(false);  // Mark button as not pressed
          myButton.drawButton();  // Draw the button in its normal state
          

          HAL_Delay(500);  // 500 ms delay

          lcdCtrl.setRotation(0);  // Set rotation (0, 1, 2, or 3)
          lcdCtrl.fillScreen(ILI9341_BLACK);
           myButton.drawButton();  // Draw the button in its normal state

           HAL_Delay(500);  // 500 ms delay for debouncing


          //lcdCtrl.print(printBuffer, sizeof(printBuffer), "Button Pressed");
        } 
        else 
        {
          myButton.press(false);  // Button not pressed
          myButton.drawButton(false);  // Redraw the button in the pressed state
        }
      }
      else 
      {
        myButton.press(false);  // No touch, button not pressed
      }
      
    }
  }

  return ret;
} 
