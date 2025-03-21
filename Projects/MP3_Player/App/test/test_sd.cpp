/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>

#include "main.h"
#include "test.h"
#include "SD.h"

int32_t test_sd()
{
  int32_t ret = 0;

  printf("-- TEST: SD Card reader\n");

  if (!SD.begin()) 
  {
      printf("ERROR: SDCard: failed to initialize.\r\n");
      return -1;
  }
  
  printf("SDCard: ls /\r\n");
  

  // UW: Files such as those created by MacOS Indexing may interfere with directory listing.
  //     You can still access individual files by their full path. Format your SD card using FAT32 
  //     (I used 4k sectors). Also, SD card is using malloc so make sure you are not running out of
  //     heap (sysmem.c).
  File root = SD.open("/");
  
  while(true)
  {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
  
    printf("\t%s %s\r\n", entry.name(), entry.isDirectory() ? "<DIR>" : "");
    entry.close();
  }
  
  root.close();

  return ret;
}
