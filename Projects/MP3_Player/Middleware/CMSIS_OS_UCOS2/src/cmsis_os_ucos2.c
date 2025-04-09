/*
 * University of Washington
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include "cmsis_os.h"
#include "ucos_ii.h"

static inline osStatus ucOSStatusAdapter(INT8U err)
{
  osStatus ret;

  switch(err)
  {
    case OS_ERR_NONE:
      ret = osOK;
      break;
    case OS_ERR_TIMEOUT:
      ret = osEventTimeout;
      break;
    case OS_ERR_PEND_ISR:
    case OS_ERR_DEL_ISR:
      ret = osErrorISR;
      break;
    case OS_ERR_PEVENT_NULL:
      ret = osErrorParameter;
      break;
    case OS_ERR_EVENT_TYPE:
      ret = osErrorValue;
      break;
    case OS_ERR_TASK_WAITING:
      ret = osErrorResource;
      break;
    default:
      ret = osErrorOS;
  }

  return ret;
} 

osMutexId osMutexCreate (const osMutexDef_t *mutex_def)
{
  // UW_TODO: Implement all wrapper functions below.
  // Note that the priority is passed through mutex_def->prio for CMSIS_RTOSv1.
  //UW_SOLUTION:
  osMutexId ret;

  if (!mutex_def)
  {
    ret = NULL;
    return NULL;
  } 
  else
  {
   ret = (osMutexId) OSMutexCreate(mutex_def->prio, NULL);
   return ret;
  } 
  // UW_TODO: Replace with the proper uCOS2 call. Return NULL on error or the mutex ID on success.
  //
  
}

osStatus osMutexWait (osMutexId mutex_id, uint32_t millisec)
{
  INT8U err;
  
  // Check the current state of the mutex. 
  OS_MUTEX_DATA md;
  err = OSMutexQuery(mutex_id, &md);

  // If the mutex is already acquired (value = 0) by the current thread (OSPrioCur), do not acquire 
  // the mutex again. 
  // See https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__MutexMgmt.html:
  // The advantage of a mutex is that it introduces thread ownership. When a thread acquires a mutex 
  // and becomes its owner, subsequent mutex acquires from that thread will succeed immediately 
  // without any latency. Thus, mutex acquires/releases can be nested.
  if (!err && !(md.OSValue == OS_FALSE && md.OSOwnerPrio == OSPrioCur))
  {
    // UW_TODO: Replace with the proper uCOS2 call.
    //UW_SOLUTION:
    OSMutexPend(mutex_id, millisec == osWaitForever ? 0 : millisec, &err);
    err = OS_ERR_NONE;
  }

  return ucOSStatusAdapter(err);
}

osStatus osMutexRelease (osMutexId mutex_id)
{
  INT8U err;
  
  // UW_TODO: Replace with the proper uCOS2 call.
  //UW_SOLUTION

  OSMutexPost(mutex_id);

  err = OS_ERR_NONE;

  return ucOSStatusAdapter(err);
}

osStatus osMutexDelete (osMutexId mutex_id)
{
  INT8U err;
  
  // UW_TODO: Replace with the proper uCOS2 call.
  //UW_SOLUTION
  OSMutexDel(mutex_id, OS_DEL_NO_PEND, &err);
  err = OS_ERR_NONE;

  return ucOSStatusAdapter(err);
}

osSemaphoreId osSemaphoreCreate (const osSemaphoreDef_t *semaphore_def, int32_t count)
{
  // UW_TODO: Replace with the proper uCOS2 call.
  //UW_SOLUTION:
  if (!semaphore_def || count < 0) 
    return NULL;
  else
    return (osSemaphoreId) OSSemCreate((INT16U)count);
  
}

int32_t osSemaphoreWait (osSemaphoreId semaphore_id, uint32_t millisec)
{
  INT8U err;
  //int32_t count;

  // UW_TODO: Replace with the proper uCOS2 call.
  //UW_SOLUTION
  err = OS_ERR_NONE;


  OSSemPend(semaphore_id, millisec == osWaitForever ? 0 : millisec, &err);
  return (err == OS_ERR_NONE) ? semaphore_id->OSEventCnt : -1;

  // if(err)
  // {
  //   count = -1;
  // }
  // else
  // {
  //   count = semaphore_id->OSEventCnt;
  // }

  // return count;
}

osStatus osSemaphoreRelease (osSemaphoreId semaphore_id)
{
  INT8U err;
  
  // UW_TODO: Replace with the proper uCOS2 call.
  OSSemPost(semaphore_id);

  err = OS_ERR_NONE;

  return ucOSStatusAdapter(err);
}

osStatus osSemaphoreDelete (osSemaphoreId semaphore_id)
{
  INT8U err;
  
  // UW_TODO: Replace with the proper uCOS2 call.
  //UW_SOLUTION:

  OSSemDel(semaphore_id, OS_DEL_NO_PEND, &err);
  err = OS_ERR_NONE;

  return ucOSStatusAdapter(err);
}
