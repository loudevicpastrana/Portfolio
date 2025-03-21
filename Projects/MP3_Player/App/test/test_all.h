/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include "main.h"
#include "ucos_ii.h"
#include "log.h"

// Declare a Mutex for SPI1 as extern here.
extern OS_EVENT *mutexSPI1;

// Task prototypes
void StartupTask(void* pdata);
void GuiTask(void* pdata);
void Mp3Task(void* pdata);

inline void SPI1Lock()
{
  //Acquire the mutexSPI1. On error, call ErrorHandler();

  INT8U err;

  OSMutexPend(mutexSPI1, 500, &err);

  if (err == OS_ERR_TIMEOUT) {
    // Handle timeout (e.g., retry, log an error, etc.)
    log_debug("Mutex Timeout");
    // Timeout occurred
            // Implement retry mechanism
    OSTimeDly(50);  // Delay before retrying
   

  }
  else if (err) 
  {
    ErrorHandler();
  }
}

inline void  SPI1Unlock()
{
  // Release the mutexSPI1. On error, call ErrorHandler();
 
  INT8U err;

  err = OSMutexPost(mutexSPI1);
  if (err) 
  {
    ErrorHandler();
  }


}
