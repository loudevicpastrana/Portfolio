/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*********************************************************************************************************
*/

#include <stm32l475xx.h>

/*
*********************************************************************************************************
*
*                                      APPLICATION CONFIGURATION
*
*                                            EXAMPLE CODE
*
* Filename : app_cfg.h
*********************************************************************************************************
*/

#ifndef  _APP_CFG_H_
#define  _APP_CFG_H_


/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/

#define  APP_CFG_SERIAL_EN                      DEF_ENABLED


/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

//task priorities
#define APP_TASK_START_PRIO                 4
#define APP_TASK_TEST1_PRIO                 5
#define APP_TASK_TEST2_PRIO                 6
#define APP_TASK_TEST3_PRIO                 7

#define APP_MUTEX_SPI1_PRIO                 3
#define APP_TASK_MP3_PRIO                   5
#define APP_TASK_GUI_PRIO                   4
#define APP_TASK_IOT_PRIO                   6
#define APP_TASK_ML_PRIO                    7

#define  OS_TASK_TMR_PRIO                (OS_LOWEST_PRIO - 2u)


/*
*********************************************************************************************************
*                                          TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_STK_SIZE            512u
#define  APP_CFG_TASK_ML_STK_SIZE              1024u

/*
*********************************************************************************************************
*                                          Interrupt configuration
*          See notes from os_cpu.h and the following link for details:
* https://www.weston-embedded.com/company/media-articles/20-cortex-m-migrate-to-new-armv7m-port
*********************************************************************************************************

CPU_CFG_KA_IPL_BOUNDARY defines the value to be used for BASEPRI, which defines the minimum priority for 
exception processing. A nonzero value for BASEPRI prevents the activation of all exceptions with the same 
or lower priority level as the BASEPRI value, so if BASEPRI is set to 3 then exceptions with priority level
3 to 15 will be disabled. Moreover, interrupt priority levels between 3-15 will be considered Kernel aware,
while the other ones will be Non-Kernel aware.
*/

// UW: See stm32l4xx_it.c for interrupts used by this application and how they compare with the
//     Kernel-Aware boundary below.
#define CPU_CFG_KA_IPL_BOUNDARY                7
#define CPU_CFG_NVIC_PRIO_BITS                 __NVIC_PRIO_BITS

#endif
