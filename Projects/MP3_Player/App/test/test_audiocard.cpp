/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>

#include "main.h"
#include "Adafruit_VS1053.h"

int32_t test_audiocard()
{
  printf("-- TEST: audiocard\n");

  int32_t ret = 0;
  Adafruit_VS1053 audioCtrl = Adafruit_VS1053();   // The Audio card controller
  
  if (!audioCtrl.begin())
  {
    printf("ERROR: Audiocard failed to initialize.\r\n");
    ret = -1;
  }
  
  if (!ret)
  {
    printf("Audiocard: BEEP!\r\n");
    audioCtrl.sineTest(0x88, 10);
  }

  return ret;
}
