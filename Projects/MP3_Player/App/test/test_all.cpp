/*
 * Loudevic Pastrana
 * 
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>
#include "test_all.h"
/************************************************************************************

   Allocate the stacks for each task.
   The maximum number of tasks the application can have is defined by OS_MAX_TASKS in os_cfg.h

************************************************************************************/
// Reserve RAM for the GUI Task:
static OS_STK GuiTaskStk[APP_CFG_TASK_START_STK_SIZE];
// Reserve RAM for the MP3 Task:
static OS_STK Mp3TaskStk[APP_CFG_TASK_START_STK_SIZE];

static OS_STK StartupTaskStk[APP_CFG_TASK_START_STK_SIZE];

// Declare the SPI1 mutex:
OS_EVENT *mutexSPI1 = 0;


/************************************************************************************

   This task is the initial task running, started by main(). It starts
   the system tick timer and creates all the other tasks. Then it converts into the MP3 task.

************************************************************************************/
void StartupTask(void* pdata)
{
  INT8U err;

  log_init();
  log_debug("StartupTask: Begin\n");
  log_debug("StartupTask: Starting timer tick\n");

  // Start the system tick
  uint32_t sysclockfreq = HAL_RCC_GetSysClockFreq();
  OS_CPU_SysTickInitFreq(sysclockfreq);

  // initialize the SPI1 Mutex.
  mutexSPI1  = OSMutexCreate(APP_MUTEX_SPI1_PRIO, &err);  // Priority inversion ceiling = 3
  
  if (err != OS_ERR_NONE) {
      printf("SPI1 Mutex creation failed! Error: %d\n", err);
  }
  
  //create the GUI task.


  err = OSTaskCreateExt(GuiTask, 
                      (void*)0, 
                      &GuiTaskStk[APP_CFG_TASK_START_STK_SIZE-1],
                      APP_TASK_GUI_PRIO, 
                      0, 
                      &GuiTaskStk[0],APP_CFG_TASK_START_STK_SIZE,NULL,
                      0);
  
  
  // This task will become the MP3 Player task.
  OSTaskNameSet(APP_TASK_MP3_PRIO, (INT8U *)"MP3", &err);
  if (err)
  {
    log_error("Cannot set Mp3Task name\n");
  }
 
  //Mp3Task(pdata);

  // Once initialization is complete, delete the startup task:
  OSTaskDel(OS_PRIO_SELF);

  while(1);
}

int32_t test_all()
{
  int32_t ret = 0;
  INT8U err;

  printf("-- TEST: ALL\n");

  // Initialize uCOS, Create the StartupTask - from HW3.

  OSInit();
  
  
  //  Create the StartupTask by calling OSTaskCreate.
  //  The stack is at Mp3TaskStk but it's very important to note that the stack grows to 
  //  lower addresses! uCOS2 requires the initial stack pointer.

  err = OSTaskCreateExt(Mp3Task, 
                       (void*)0, 
                       &Mp3TaskStk[APP_CFG_TASK_START_STK_SIZE-1],
                       APP_TASK_MP3_PRIO,
                       0, 
                       &Mp3TaskStk[0],
                       APP_CFG_TASK_START_STK_SIZE,NULL,0);
  
  if (err) 
  {
    log_error("MP3 task creation failed: %d\n", err);
  }

  err = OSTaskCreateExt(
    StartupTask,                              // Task function
    (void *)0,                                // No argument is passed
    &StartupTaskStk[APP_CFG_TASK_START_STK_SIZE - 1], // Pointer to the top of the stack (stacks grow downward)
    1,                        // Task priority (lower value = higher priority)
    0,                                        // Task ID (often set to 0 if not used)
    &StartupTaskStk[0],                       // Pointer to the bottom of the stack
    APP_CFG_TASK_START_STK_SIZE,                    // Stack size in OS_STK units
    (void *)0,                                // No extra TCB storage
    OS_TASK_OPT_STK_CLR                       // Options (e.g., clear stack for debugging)
);


  // Start uCOS2.
  OSStart();
    
  printf("ERROR: RTOS should never reach this point!\n");
  return ret;
}
