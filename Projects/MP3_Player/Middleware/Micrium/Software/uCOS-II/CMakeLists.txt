set(TARGET ucos2)

set(SOURCES
    ./Source/os_dbg_r.c
    ./Source/os_sem.c
    ./Source/os_mbox.c
    #./Source/ucos_ii.c
    ./Source/os_q.c
    ./Source/os_mutex.c
    ./Source/os_time.c
    ./Source/os_core.c
    ./Source/os_task.c
    ./Source/os_flag.c
    ./Source/os_tmr.c
    ./Source/os_mem.c
    ./ARM-Cortex-M4/os_cpu_c.c
    ./ARM-Cortex-M4/GNU/os_dbg.c
    ./ARM-Cortex-M4/GNU/os_cpu_a.S
    ../../../../Configuration/uCOS/app_hooks.c
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
        ./Source
        ./ARM-Cortex-M4/GNU
        ../../../../Configuration/uCOS     # uCOS2 Configuration
        ../../../../Drivers/CMSIS/Core/Include          # Custom port for this CortexM implementation - used by app_cfg.h
        ../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include
)

target_link_libraries(${TARGET}
    drivers
)
