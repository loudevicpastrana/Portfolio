/*
 * University of Washington
 * Certificate in Embedded and Real-Time Systems
 *
 * Debugger
 * (c) 2021 Cristian Pop
 */

#include <stdint.h>
#include <memory.h>
#include <stdio.h>
#include "main.h"

#define DEBUG_CODE_DUMP_LOCATION    0x10000000
#define DEBUG_REGISTERS_SAVED       15
#define DEBUG_STACK_WORDS_SAVED     0       // Important: do not exceed the bottom of the stack!
#define DEBUG_TOTAL_WORDS_SAVED     (DEBUG_REGISTERS_SAVED + DEBUG_STACK_WORDS_SAVED)
#define DEBUG_DUMP_MARKER           0xDEADBEEF
/*
 *
 * Part of a fault exception handler. Prints the given register values.
 * pc: the value of the program counter when the fault occurred.
 * lr: the value of the link register when the fault occurred.
 *
 */
void CoreDump(int32_t* registers_frame)
{
    int32_t* debug_buffer = (int32_t*)DEBUG_CODE_DUMP_LOCATION;

    *debug_buffer++ = DEBUG_DUMP_MARKER;  // Marker to identify valid crash dump

    // Save important registers
    *debug_buffer++ = registers_frame[0]; // PC
    *debug_buffer++ = registers_frame[1]; // LR
    *debug_buffer++ = registers_frame[2]; // R0
    *debug_buffer++ = registers_frame[3]; // R1
    *debug_buffer++ = registers_frame[4]; // R2
    *debug_buffer++ = registers_frame[5]; // R3
    *debug_buffer++ = registers_frame[6]; // R12
    *debug_buffer++ = registers_frame[7]; // LR (Link Register) for exceptions, not general purpose LR
    *debug_buffer++ = registers_frame[8]; // PC during the exception

    *debug_buffer++ = *((volatile int32_t*)0xE000ED28);  // CFSR (Configurable Fault Status Register)
    *debug_buffer++ = *((volatile int32_t*)0xE000ED2C);  // HFSR (HardFault Status Register)
    *debug_buffer++ = *((volatile int32_t*)0xE000ED34);  // MMFAR (Memory Management Fault Address Register)
    *debug_buffer++ = *((volatile int32_t*)0xE000ED38);  // BFAR (Bus Fault Address Register)
    *debug_buffer++ = *((volatile int32_t*)0xE000ED3C);  // AFSR (Auxiliary Fault Status Register)

    // Assuming PSR (Program Status Register) is saved at a certain offset:
    *debug_buffer++ = registers_frame[9]; // PSR

    // Optional: Copy additional stack content for debugging
    memcpy(debug_buffer, registers_frame, DEBUG_TOTAL_WORDS_SAVED * sizeof(int32_t));

    // Reset system (reboot)
    NVIC_SystemReset();

    // Unreachable code
    while (1) {}
}


int CheckCoreDump()
{
    int32_t* debug_buffer = (int32_t*)DEBUG_CODE_DUMP_LOCATION;

    if (debug_buffer[0] == (int32_t)DEBUG_DUMP_MARKER)
    {
        debug_buffer[0] = 0;

        printf("The system rebooted: crash-dump present!\r\n");
        
        printf("Crash Dump: PC = 0x%08lX\t, LR = 0x%08lX\r\n", debug_buffer[1], debug_buffer[2]);

        // Print R0 - R3 (first 4 general-purpose registers)
        for (int i = 0; i < 4; i++) {
            printf("R%d = 0x%08lX\r\n", i, debug_buffer[3 + i]);
        }

        // Print R12
        printf("R12 = 0x%08lX\r\n", debug_buffer[7]);

        // Print LR (for exceptions, not general purpose)
        printf("LR (exception LR) = 0x%08lX\r\n", debug_buffer[8]);

        // Print the second PC (where the exception was triggered)
        printf("PC (exception PC) = 0x%08lX\r\n", debug_buffer[9]);

        // Print PSR (Program Status Register)
        printf("PSR = 0x%08lX\r\n", debug_buffer[10]);

        // Print CFSR, HFSR, MMFAR, BFAR, AFSR (fault status registers)
        printf("CFSR = 0x%08lX\r\n", debug_buffer[11]);
        printf("HFSR = 0x%08lX\r\n", debug_buffer[12]);
        printf("MMFAR = 0x%08lX\r\n", debug_buffer[13]);
        printf("BFAR = 0x%08lX\r\n", debug_buffer[14]);
        printf("AFSR = 0x%08lX\r\n", debug_buffer[15]);

        // UW_TODO: Print the contents of debug_buffer[] (length = DEBUG_TOTAL_WORDS_SAVED).
        // E.g. R0: <value from debug_buffer[1].
        
        // UW_TODO: For EXTRA CREDIT only: you will need to change DEBUG_REGISTERS_SAVED and 
        //          DEBUG_TOTAL_WORDS_SAVED. If you do change DEBUG_TOTAL_WORDS_SAVED be careful not
        //          to exceed the bottom of the stack and cause a fault within CoreDump!
        printf("\n--------\n\n");
        return 1;
    }

    return 0;
}
