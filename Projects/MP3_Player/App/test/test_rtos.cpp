/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>
#include <stdarg.h>
#include "main.h"
#include "ucos_ii.h"
#include "log.h"

// Allocate a stack for the startup task
static OS_STK StartupStk[APP_CFG_TASK_START_STK_SIZE];

/************************************************************************************

   Allocate the stacks for each task.
   The maximum of tasks the application can have is defined by OS_MAX_TASKS in os_cfg.h

************************************************************************************/

static OS_STK   Task1Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK   Task2Stk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK   Task3Stk[APP_CFG_TASK_START_STK_SIZE];


// Task prototypes
static void ProduceLower(void* pdata);
static void ProduceUpper(void* pdata);
static void Consume(void* pdata);

OS_EVENT *qMsg;                 // pointer to a uCOS message queue
#define QMAXENTRIES 4            // maximum entries in the queue
void * qMsgVPtrs[QMAXENTRIES];   // an array of void pointers which is the actual queue

/************************************************************************************

   This task is the initial task running, started by main(). It starts
   the system tick timer and creates all the other tasks. Then it deletes itself.

************************************************************************************/
static void StartupTask(void* pdata)
{
  INT8U err;
  log_init();
  log_debug("StartupTask()\n");

  // Cortex-Debug XRTOS does not pick this up correctly:
  // https://github.com/Marus/cortex-debug/issues/798
  OSTaskNameSet(OSPrioCur, (INT8U*)"StartupTask()", &err);
  if (err) 
  {
    log_error("Failed to set task name: %d\n", err);
  }

  // Start the system tick
  uint32_t sysclockfreq = HAL_RCC_GetSysClockFreq();
  OS_CPU_SysTickInitFreq(sysclockfreq);

  // Enable OS Statistics
  OSStatInit();
  qMsg = OSQCreate(qMsgVPtrs, QMAXENTRIES);

  log_debug("OSDelay(1000)\n");
  OSTimeDly(1000);
  log_debug("HAL_Delay(1000)\n");
  OSTimeDly(1000);

  // Create the the test tasks
  log_debug("Creating 3 tasks...\n");

  // The maximum of tasks the application can have is defined by OS_MAX_TASKS in os_cfg.h
  err = OSTaskCreateExt(
    ProduceLower, 
    (void*)0, 
    &Task1Stk[APP_CFG_TASK_START_STK_SIZE-1], 
    APP_TASK_TEST1_PRIO, 
    APP_TASK_TEST1_PRIO, 
    &Task1Stk[0], 
    APP_CFG_TASK_START_STK_SIZE,
    NULL, 
    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_SAVE_FP);

  if (err) 
  {
    log_error("ProduceLower task creation failed: %d\n", err);
  }

  err = OSTaskCreateExt(
    ProduceUpper, 
    (void*)0, 
    &Task2Stk[APP_CFG_TASK_START_STK_SIZE-1], 
    APP_TASK_TEST2_PRIO, 
    APP_TASK_TEST2_PRIO, 
    &Task2Stk[0], 
    APP_CFG_TASK_START_STK_SIZE,
    NULL, 
    OS_TASK_OPT_STK_CHK);
  if (err) 
  {
    log_error("ProduceUpper task creation failed: %d\n", err);
  }

  // UW: CortexDebug with XRTOS hangs if stack information is not given. 
  //     https://github.com/Marus/cortex-debug/issues/797
  //     Instead of the following, use the extended version.
  // err = OSTaskCreate(Consume, (void*)0, &Task3Stk[APP_CFG_TASK_START_STK_SIZE-1], APP_TASK_TEST3_PRIO);
  err = OSTaskCreateExt(
    Consume, 
    (void*)0, 
    &Task3Stk[APP_CFG_TASK_START_STK_SIZE-1], 
    APP_TASK_TEST3_PRIO, 
    APP_TASK_TEST3_PRIO, 
    &Task3Stk[0], 
    APP_CFG_TASK_START_STK_SIZE,
    NULL, 
    OS_TASK_OPT_STK_CHK);

  if (err) 
  {
    log_error("Consume creation failed: %d\n", err);
  }

  // Delete ourselves, letting the work be done in the new tasks.

  // UW: Inspect OSTCBStkUsed for this task to determine the optimum stack size. 
  OS_STK_DATA stack_info;
  err = OSTaskStkChk(OS_PRIO_SELF, &stack_info);
  if (err) 
  {
    log_error("Failed to retrieve stack statistics: %d\n", err);
  }
  else
  {
    log_debug("StartupTask stack statistics: used=%d free=%d\n", stack_info.OSUsed, stack_info.OSFree);
  }

  log_debug("Startup task deleting itself.\n");
	OSTaskDel(OS_PRIO_SELF);
}

/************************************************************************************

   Task ProduceLower posts lower case letters to the queue

************************************************************************************/
static void ProduceLower(void* pdata)
{
  INT8U err;
  int i = 0;
  char message[100];

  while(i < 26)
  {
    message[i] = i % 26 + 'a';
    do
    {
      err = OSQPost(qMsg, &message[i]);
      if (err) 
      {
        OSTimeDly(1);
      }
    } while (err == OS_ERR_Q_FULL);
    
    log_debug("1: %c\n", message[i]);
    i++;
    if (i>26) OSTimeDly(1000);
  }

  log_error("1: done\n");
  // UW: Inspect OSTCBStkUsed for this task to determine the optimum stack size. 
  OS_STK_DATA stack_info;
  err = OSTaskStkChk(OS_PRIO_SELF, &stack_info);
  if (err) 
  {
    log_error("Failed to retrieve stack statistics: %d\n", err);
  }
  else
  {
    log_debug("ProduceLower stack statistics: used=%d free=%d\n", stack_info.OSUsed, stack_info.OSFree);
  }

	OSTaskDel(OS_PRIO_SELF);
}

/************************************************************************************

   Task ProduceUpper posts upper case letters in the queue

************************************************************************************/
static void ProduceUpper(void* pdata)
{
  INT8U err;
  int i = 0;
  char message[100];

  while(i < 26)
  {
    message[i] = i % 26 + 'A';
    do
    {
      err = OSQPost(qMsg, &message[i]);
      if (err)
      {
        OSTimeDly(1);
      }
    } while (err == OS_ERR_Q_FULL);

    log_debug("2: %c\n", message[i]);

    i++;
    if (i>26) OSTimeDly(1000);
  }

  log_error("2: done\n");
  // UW: Inspect OSTCBStkUsed for this task to determine the optimum stack size. 
  OS_STK_DATA stack_info;
  err = OSTaskStkChk(OS_PRIO_SELF, &stack_info);
  if (err) 
  {
    log_error("Failed to retrieve stack statistics: %d\n", err);
  }
  else
  {
    log_debug("ProduceLower stack statistics: used=%d free=%d\n", stack_info.OSUsed, stack_info.OSFree);
  }
  
	OSTaskDel(OS_PRIO_SELF);
}

/************************************************************************************

   Task Consume consumes messages from the queue

************************************************************************************/
static void Consume(void* pdata)
{
  INT8U err;
  int i = 3000;

  while(1)
  {
    char* message = (char*)OSQPend(qMsg, 0, &err);
    i++;
    log_debug("3:%c\n", *message);
  }
}

int32_t test_rtos()
{
  int32_t ret = 0;
  
  printf("-- TEST: uCOS-2 RTOS\n");

  INT8U err;

  printf("OSInit()\n");
  OSInit();

  err = OSTaskCreateExt(
    StartupTask,
    (void*)0,
    &StartupStk[APP_CFG_TASK_START_STK_SIZE -1],
    APP_TASK_START_PRIO,
    APP_TASK_START_PRIO,
    &StartupStk[0],
    APP_CFG_TASK_START_STK_SIZE,
    NULL,
    OS_TASK_OPT_STK_CHK);

  if (err) 
  {
    printf("OSTaskCreate failed: %d\n", err);
  }

  OSStart();

  printf("ERROR: RTOS should never reach this point!\n");
  return ret;
}
