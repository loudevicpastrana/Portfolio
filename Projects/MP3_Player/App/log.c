/*
 * Loudevic Pastrana
 * 
 *
 * (c) 2021 Cristian Pop
 * 
 * Smaller version of https://github.com/kala13x/slog
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include "log.h"
#include "main.h"
#include "ucos_ii.h"

static OS_EVENT *uartMutex;

void log_init()
{
  INT8U err;
  uartMutex = OSMutexCreate(OS_PRIO_MUTEX_CEIL_DIS, &err);
  if (err) 
  { 
    ErrorHandler();
  }
}

void log_display(slog_flag_t level, const char *pFormat, ...)
{
  INT8U err;
  OSMutexPend(uartMutex, 0, &err);
  if (err) 
  {
    ErrorHandler();
  }

  int task = OSPrioCur;

  switch(level)
  {
    case LOG_DEBUG:
    printf("(%2d) %s[DEBUG] %s", task, LOG_COLOR_BLUE, LOG_COLOR_RESET);
    break;

    case LOG_INFO:
    printf("(%2d) %s[INFO ] %s", task, LOG_COLOR_GREEN, LOG_COLOR_RESET);
    break;

    case LOG_ERROR:
    printf("(%2d) %s[ERROR] %s", task, LOG_COLOR_RED, LOG_COLOR_RESET);
    break;

    default:
    printf("(?) ");
  }

  va_list ap;
  va_start(ap, pFormat);
  vprintf(pFormat, ap);
  va_end(ap);
  
  err = OSMutexPost(uartMutex);
  if (err) 
  {
    ErrorHandler();
  }
}

void log_destroy()
{
  INT8U err;
  OSMutexDel(uartMutex, OS_DEL_NO_PEND, &err);
  if (err) 
  {
    ErrorHandler();
  }
}
