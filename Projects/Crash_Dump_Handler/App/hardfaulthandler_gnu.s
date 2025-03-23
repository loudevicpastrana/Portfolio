/**
 * Define a name for this module of code/data (ie the code in this file, only 1 modules per file is allowed)
 * this module will be placed at the boot vector of the Cortex by the linker (see the linker cmd line for explanation)
*/

 /* See https://sourceware.org/binutils/docs-2.35/as/ARM-Directives.html#ARM-Directives for more information. */

.syntax unified
	.cpu cortex-m4
	.fpu fpv4-sp-d16
	.thumb

.extern     CoreDump
.global	HardFault_Handler
/* HardFault_Handler extracts information from the CPU (registers and stack) and saves them
   to the stack in a particular format. These are passed to CoreDump for processing. */

.equ  CFSR,       0xE000ED28
.equ  HFSR,       0xE000ED2C
.equ  DFSR,       0xE000ED30
.equ  MMFAR,      0xE000ED34
.equ  BFAR,       0xE000ED38
.equ  AFSR,       0xE000ED3C

.section	.text.HardFault_Handler
.type	HardFault_Handler, %function
HardFault_Handler:

MRS R0, MSP                 // Get main stack pointer (exception stack)
LDR R1, [R0, #24]           // Load Program Counter (PC) from stack (saved by hardware)
LDR R2, [R0, #28]           // Load Link Register (LR) from stack (saved by hardware)

 // Load CFSR, HFSR, MMFAR, BFAR, AFSR
LDR R3, =CFSR               // Load address of CFSR register
LDR R3, [R3]                // Load the value of CFSR
LDR R4, =HFSR               // Load address of HFSR register
LDR R4, [R4]                // Load the value of HFSR
LDR R5, =MMFAR              // Load address of MMFAR register
LDR R5, [R5]                // Load the value of MMFAR
LDR R6, =BFAR               // Load address of BFAR register
LDR R6, [R6]                // Load the value of BFAR
LDR R7, =AFSR               // Load address of AFSR register
LDR R7, [R7]                // Load the value of AFSR

// Call the C function to save crash dump
BL CoreDump
/* UW_TODO:

Place a breakpoint here and step through assembly, right before the hard fault is called (you can do this with Cortex-Debug Preview with the new Set Force Disassembly feature or in GDB using `stepi` or `si`).
Inspect and save registers: 

```
info registers
r0             0xb                 0xb
r1             0x22400000          0x22400000
r2             0x12000000          0x12000000
r3             0x41000000          0x41000000
r4             0x20000c1c          0x20000c1c
r5             0x0                 0x0
r6             0x0                 0x0
r7             0x20017fd0          0x20017fd0
r8             0x0                 0x0
r9             0x0                 0x0
r10            0x0                 0x0
r11            0x0                 0x0
r12            0xb                 0xb
sp             0x20017fd0          0x20017fd0
lr             0x8008665           0x8008665
pc             0x8008632           0x8008632 <GenerateFault+14>
xPSR           0x21010000          0x21010000
fpscr          0x0                 0x0
msp            0x20017fd0          0x20017fd0
psp            0x0                 0x0
primask        0x0                 0x0
basepri        0x0                 0x0
faultmask      0x0                 0x0
control        0x4                 0x4
```

Here is the annotated output (an example only - your values will be different):
```
x/8x $sp
            R0          R1          R2          R3
0x20017f68: 0x0000000b  0x22400000  0x12000000  0x41000000
            R12         LR          PC          xPSR
0x20017f78: 0x0000000b  0x08008665  0x08008632  0x21010000
```

Example of printing the values for R0, R1, R2 and R3:
```
        printf("\nR0 -  R3:\t");
        for (int i = 0; i < 4; i++)
        {
            printf("%08lx\t", debug_buffer[15 + i]);
        }
```

*/

      // Assuming MSP won't fault with stacking error (SCB->CFSR->STKERR)

      // UW_TODO: R0 - R3, R12, LR, PC, xPSR have been already saved by the MCU on any ISR (8 words).
      //          Halt with a debugger and match the registers on the stack with the ones in the
      //          registers view.


      // UW_TODO: For EXTRA CREDIT only: Save R4-R11 (8 words)
      
      // UW_TODO: For EXTRA CREDIT only: Save Fault status and address registers (6 words).
      //          For convenience, I've defined pointers to them above as the =CFSR, =HFSR constants etc.

      // UW_TODO: Consult the ARM ABI convention (Course 1, ARM book, etc). Prepare to call 
      //          void CoreDump(int32_t* registers_frame).
      
      // UW_TODO: Set the only argument (registers_frame).

      // UW_TODO: PUSH LR on the stack since we're calling another function - makes the debugger happy.
      
      // Call CoreDump() to save all values.

      // CoreDump should reboot. Below code should be unreachable.
loop:
      B loop
