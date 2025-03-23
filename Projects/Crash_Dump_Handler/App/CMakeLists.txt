set(TARGET crash_dump_test)

set(SOURCES
    # App template:
    main.cpp
    startup_stm32l475xx.s   # Entrypoint and Interrupt Vector Table with default handlers
    stm32l4xx_it.c          # Interrupt vector overrides
    system_stm32l4xx.c      # L4XX System Init
    syscalls.c              # required by libc / stdio
    sysmem.c                # required by libc / malloc
    
    # Debugger:
    debugger.c
    test/test_debugger.c
    hardfaulthandler_gnu.s  # UW Replaces code in stm32l4xx_it.c

    # UI
    test/test_ui.cpp
)

set(LINKER_LD_FILE ./STM32L475VGTx_FLASH.ld)

add_executable(${TARGET}
    ${SOURCES}
)

# For .weak and --whole-archive see 
# https://stackoverflow.com/questions/42269765/irq-symbol-defined-in-static-library-does-not-override-weak-irq-definition-from
# https://stackoverflow.com/questions/53071878/using-whole-archive-linker-option-with-cmake-and-libraries-with-other-library

target_link_libraries(${TARGET}
    -Wl,--whole-archive    
    drivers
    -Wl,--no-whole-archive
    adafruit_ili9341
    adafruit_ft6206
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
        ./test
)

# Convert errors to warnings:
# target_compile_options(${TARGET} PRIVATE -Wno-error=double-promotion -Wno-error=missing-field-initializers)

# Disable warnings as well:
target_compile_options(${TARGET} PRIVATE -Wno-double-promotion -Wno-missing-field-initializers)

add_map_file(${TARGET} ${TARGET}.map)

set_target_linker(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/${LINKER_LD_FILE})

post_build(${TARGET})
