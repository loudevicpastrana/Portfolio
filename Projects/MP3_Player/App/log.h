/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 * 
 * Smaller version of https://github.com/kala13x/slog
 */

#ifndef LOG_H
#define LOG_H


#ifdef __cplusplus
 extern "C" {
#endif

// Comment this to disable UART logging.
#define LOG_ENABLE

// Comment this to disable coloring
#define LOG_COLOR

/* Log level flags */
typedef enum
{
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_ERROR = 2
} slog_flag_t;

/* Supported colors */
#define LOG_COLOR_NORMAL       "\x1B[0m"
#define LOG_COLOR_RED          "\x1B[31m"
#define LOG_COLOR_GREEN        "\x1B[32m"
#define LOG_COLOR_YELLOW       "\x1B[33m"
#define LOG_COLOR_BLUE         "\x1B[34m"
#define LOG_COLOR_MAGENTA      "\x1B[35m"
#define LOG_COLOR_CYAN         "\x1B[36m"
#define LOG_COLOR_WHITE        "\x1B[37m"
#define LOG_COLOR_RESET        "\033[0m"

void log_init_internal();
void log_display(slog_flag_t level, const char *pFormat, ...);
void log_destroy_internal();

#ifdef LOG_ENABLE
  #define log_debug(...) \
      log_display(LOG_DEBUG, __VA_ARGS__)

  #define log_info(...) \
      log_display(LOG_INFO, __VA_ARGS__)

  #define log_error(...) \
      log_display(LOG_ERROR, __VA_ARGS__)
  #define log_init log_init_internal
  #define log_destroy log_destroy_internal

#else
  #define log_debug(...)
  #define log_error(...)
  #define log_init
  #define log_destroy
#endif

#ifdef __cplusplus
 }
#endif

#endif //!LOG_H
