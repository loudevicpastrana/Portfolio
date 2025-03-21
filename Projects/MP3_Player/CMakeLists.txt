cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_C_STANDARD 99)

# Set the toolchain if not defined
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/arm-gcc-cortex-m4.cmake")
endif()

include(${CMAKE_SOURCE_DIR}/cmake/utilities.cmake)

# Define the Project
project(uw_rtos_project C CXX ASM)

add_subdirectory(Drivers)   # Drivers: CMSIS, HAL, BSP, daughterboard (LCD, Touchscreen, Audio Interface)
add_subdirectory(Middleware/Micrium/Software/uCOS-II)        # Middleware: uCOS-2
add_subdirectory(App)       # Application
