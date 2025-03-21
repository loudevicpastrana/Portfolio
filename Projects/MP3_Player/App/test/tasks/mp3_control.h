#ifndef MP3_CONTROL_H
#define MP3_CONTROL_H

#include "os.h"  // Include Micrium OS headers

#define MP3_QUEUE_SIZE  10  // Maximum number of messages in the queue

// Enum for MP3 control commands
typedef enum {
    MP3_CMD_STOP,
    MP3_CMD_PLAY,
    MP3_CMD_PAUSE,
    MP3_CMD_RESUME,
    MP3_CMD_NEXT,
    MP3_CMD_PREV
} Mp3Command_t;

// Declare the message queue
extern OS_EVENT *mp3CmdQueue;

extern int currentSongIndex;

#define MAX_FILES 5
#define MAX_FILENAME_LENGTH 32 

extern char mp3Files[MAX_FILES][MAX_FILENAME_LENGTH];
#endif  // MP3_CONTROL_H
