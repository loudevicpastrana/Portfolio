/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include "test_all.h"

#include <SD.h>
#include <Adafruit_VS1053.h>
#include "mp3_control.h"  // Include the header
#include "os.h"  // Include Micrium OS headers
#include "ucos_ii.h"
#include <string.h>
#include <string>


OS_EVENT  *mp3CmdQueue; 

static  void      *MessageStorage[MP3_QUEUE_SIZE];


#define MP3_SD_BUFFER_SIZE 4096
#define MP3_AUDIOCARD_BUFFER_SIZE 32

OS_ERR err;
Mp3Command_t* cmd;
//OS_MSG_SIZE msg_size;

int currentSongIndex = 1;   // Index of the current song



char mp3Files[MAX_FILES][MAX_FILENAME_LENGTH];
int fileCount = 0;


Adafruit_VS1053 audioCtrl = Adafruit_VS1053();   // The Audio card controller (SPI1)


static void Mp3StreamStart()
{
  // reset playback
  //audioCtrl.softReset();
  SPI1Lock();  
  audioCtrl.sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_LINE1 | VS1053_MODE_SM_SDINEW |
                                VS1053_MODE_SM_LAYER12);
  // resync
  audioCtrl.sciWrite(VS1053_REG_WRAMADDR, 0x1e29);
  audioCtrl.sciWrite(VS1053_REG_WRAM, 0);

  // As explained in datasheet, set twice 0 in REG_DECODETIME to set time back
  // to 0
  audioCtrl.sciWrite(VS1053_REG_DECODETIME, 0x00);
  audioCtrl.sciWrite(VS1053_REG_DECODETIME, 0x00);
        
  audioCtrl.setVolume(40, 40);
  SPI1Unlock();
}

/**
 * @brief Stops streaming and all audio.
 * @note NOT thread-safe: The SPI lock must be taken before calling this function.
 */
static void Mp3StreamStop()
{
  // cancel all playback
  SPI1Lock();
  audioCtrl.sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_LINE1 | VS1053_MODE_SM_SDINEW |
                                VS1053_MODE_SM_CANCEL);
  SPI1Unlock();
}

static void Mp3StreamFile(const char* fileName) {
    audioCtrl.softReset();
    Mp3StreamStart();

    File fp = SD.open(fileName);
    if (!fp) {
        log_error("Failed to open %s\n", fileName);
        return;
    }

    static uint8_t mp3_buffer[MP3_SD_BUFFER_SIZE];

    bool stopRequested = false;  // Flag to track if stop was requested

    while (!stopRequested) {  // Loop should exit if stop is requested
        
        void *p_msg = OSQAccept(mp3CmdQueue, &err);  // Accept a message from the queue
        if (err == OS_ERR_NONE && p_msg != NULL) {
            cmd = (Mp3Command_t*) p_msg;
            if (*cmd == MP3_CMD_STOP || *cmd == MP3_CMD_NEXT) {
                stopRequested = true;  // Set flag to stop playback
                break;  // Exit the loop and stop streaming
            }
           
        }

        // Lock SPI only for reading
        SPI1Lock();
        int readBytes = fp.read(mp3_buffer, MP3_SD_BUFFER_SIZE);
        SPI1Unlock();

        if (readBytes <= 0) {
            log_debug("End of file or read error.");
            break;
        }

        int sent = 0;
        while (sent < readBytes) {
            int delta = MP3_AUDIOCARD_BUFFER_SIZE;

            if (sent + delta > readBytes) {
                delta = readBytes - sent;
            }

            // Wait until VS1053 is ready for data, but allow other tasks to run
            while (!audioCtrl.readyForData()) {
                OSTimeDly(5);  // Yield to GUI task
            }

            // Lock SPI only for sending audio data
            SPI1Lock();
            audioCtrl.playData(mp3_buffer + sent, delta);
            SPI1Unlock();

            sent += delta;

            // Yield to GUI task after processing a small chunk
            OSTimeDly(1);
        }
    }

    //Mp3StreamStop();
    //fp.close();
}

void LoadMp3Files() {
    // Open the root directory
    File rootDir = SD.open("/");
    if (!rootDir) {
      log_error("Failed to open root directory.");
      return;
    }
  
    // Rewind the directory to start reading files
    rootDir.rewindDirectory();
  
    while(true)
    {
      File entry =  rootDir.openNextFile();
      if (! entry) {
        // no more files
        break;
      }
      
      int nLength = 0;

      const char *filename = entry.name();  
      if (strstr(filename, ".mp3") || strstr(filename, ".MP3")) {
        fileCount++;
        nLength = strlen(filename);
        strncpy(mp3Files[fileCount], filename, nLength);
        //memcpy(mp3Files[fileCount], filename, nLength); 
      }

      //printf("\t%s %s\r\n", entry.name(), entry.isDirectory() ? "<DIR>" : "");
      printf("\t%s \r\n", filename);


      entry.close();
    }
    
    // Close the root directory
    rootDir.close();
  
    // If no MP3 files were found
    if (fileCount == 0) {
      log_debug("No MP3 files found.\n");
    }
  }
  


// Function to play the next song
void PlayNextSong() {


    if (fileCount == 0) {
        log_error("No MP3 files found on SD card.");
        return;
    }
 
    currentSongIndex++;  // Move to next song
    if (currentSongIndex > fileCount) {
        currentSongIndex = 0;  // Wrap around to the first song
    }

    log_debug("Playing next song: %s", mp3Files[currentSongIndex]);
    Mp3StreamFile(mp3Files[currentSongIndex]);
}

void Mp3Task(void* pdata) {
    OS_ERR err;
    Mp3Command_t *cmd;
    void *p_msg;
    //CPU_INT16U msg_size;  // Declare msg_size to store the message size

    log_debug("MP3Task: starting\n");
    
    
    mp3CmdQueue = OSQCreate(&MessageStorage[0], MP3_QUEUE_SIZE); 

    if (mp3CmdQueue == ((void *)0)) { 
        /* Failed to create message box   */ 
        /* Return error to caller?        */ 
        log_error("Failed to create MP3 command queue");
    }
    


    SPI1Lock();
    if (!audioCtrl.begin()) {
        log_error("Failed to initialize audioCtrl");
        SPI1Unlock();
        ErrorHandler();
        return;
    }

    if (!SD.begin()) {
        log_error("ERROR: SD card initialization failed!");
        SPI1Unlock();
        return;
    }
    SPI1Unlock();

    LoadMp3Files();  

    p_msg =  OSQPend(mp3CmdQueue, 0, &err); 

    while (1) {

        //p_msg =  OSQAccept(mp3CmdQueue, &err); 
         

        if (err == OS_ERR_NONE) {
           cmd = (Mp3Command_t *)p_msg;
            switch (*cmd) {
                case MP3_CMD_STOP:{
                    Mp3StreamStop();
                    break;}
                case MP3_CMD_PLAY: {
                    // const char* fileName = "slowlife.mp3"; // You could dynamically set this here
                    // log_debug("Begin streaming sound file: %s\n", fileName);
                    Mp3StreamFile(mp3Files[fileCount]);  // Pass the filename dynamically
                    break;
                }

                case MP3_CMD_NEXT:
                    Mp3StreamStop();
                    PlayNextSong();
                    break;
                // case MP3_CMD_PREV:
                //     PlayPreviousSong();
                //     break;
                default:
                    break;
            }
        }
    }
}
