Crash Handler

## Part 1 - Crash Dump Handler
 This is a diagnostic hard fault exception handler. In debug systems it's common to have an infinite loop handling all fault handlers. This isn't practical when the device is deployed in the field. It's common for devices to collect information and then reboot to continue operation as soon as possible.

During a fault, you may not assume that systems such as UART still work (what if the crash is because of the UART driver?). Instead, save all information that will help you debug into a pre-allocated memory region. On reboot (in `main`), check if the memory region contains crash-dump information and display it. 

This is useful in real-world scenarios where we wouldn't be able to continue execution (and risk further corruption). If the board has IoT capabilities, you can use this module to collect the information within your cloud for analysis and over-the-air updates.



```
-- TEST: debugger
The system rebooted: crash-dump present!

R0 -  R4:       0000000b        22400000        12000000        41000000
R5 -  R9:       00000000        00000000        20017fd0        00000000
R9 - R12:       00000000        00000000        00000000        0000000b

xPSR:   21030000        PC:     080087b6        LR:     080087e9
CFSR    00008200        HFSR    40000000        DFSR    0000000a
MMFAR   41000000        BFAR    41000000        AFSR    00000000

Stack:
00000000        00000000        00000000        00000000
--------
```