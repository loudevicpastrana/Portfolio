/*
 * University of Washington
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include "main.h"
#include <stdio.h>

extern void HardFault_Handler(void);
// Generates a fault exception
void GenerateFault()
{
    /* UW_TODO : Modify the code to cause a data fetch bus fault by reading */
    /* memory at an invalid address. Look this up within the MCU's memory map document.*/
    int *a = (int*) 0xFFFFFFFF;
    int b = *a;
    b = b; // This line just avoids a compiler warning

    printf("\nFail! Should not arrive here.\n");
    while (1);
}

extern int CheckCoreDump();

int test_debugger() 
{
    int ret;
    printf("-- TEST: debugger\n");

    if (!CheckCoreDump())
    {
        printf("TEST CRASH\n");
        GenerateFault();
        printf("\nFail! Should not arrive here.\n");
        ret = 1;
    }
    else
    {
        printf("\nOK.\n");
        ret = 0;
    }

    return ret;
}
