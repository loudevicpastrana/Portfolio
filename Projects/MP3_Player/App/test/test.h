/*
 * Loudevic Pastrana
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#ifndef  _TEST_H_
#define  _TEST_H_

#include <stdint.h>

extern "C" int32_t test_debugger();
int32_t test_rtos();
int32_t test_sd();
int32_t test_audiocard();
int32_t test_ui();
int32_t test_all();

#endif // !_TEST_H_
