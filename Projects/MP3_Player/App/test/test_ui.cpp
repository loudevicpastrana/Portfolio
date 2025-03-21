/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>

#include "main.h"

#include "Adafruit_ILI9341.h"
#include "Adafruit_FT6206.h"

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
  }

  return ret;
}
