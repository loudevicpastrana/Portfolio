# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

# Original at: https://github.com/azure-rtos/getting-started/

set(CMAKE_SYSTEM_NAME Generic)

set(TARGET_TRIPLET "arm-none-eabi-")

# default to Debug build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release." FORCE)
endif()

# do some windows specific logic
if(WIN32)
    set(TOOLCHAIN_EXT ".exe")
else()
    set(TOOLCHAIN_EXT "")
endif(WIN32)

find_program(COMPILER_ON_PATH "${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT}")

if(DEFINED ENV{ARM_GCC_PATH}) 
    # use the environment variable first    
    file(TO_CMAKE_PATH $ENV{ARM_GCC_PATH} ARM_TOOLCHAIN_PATH)
    message(STATUS "Using ENV variable ARM_GCC_PATH = ${ARM_TOOLCHAIN_PATH}")
elseif(COMPILER_ON_PATH) 
    # then check on the current path
    get_filename_component(ARM_TOOLCHAIN_PATH ${COMPILER_ON_PATH} DIRECTORY)
    message(STATUS "Using ARM GCC from path = ${ARM_TOOLCHAIN_PATH}")
else()
    message(FATAL_ERROR "Unable to find ARM GCC. Either add to your PATH, or define ARM_GCC_PATH to the compiler location")
endif()

# Perform compiler test with the static library
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER    ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT} CACHE STRING "")
set(CMAKE_CXX_COMPILER  ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}g++${TOOLCHAIN_EXT} CACHE STRING "")
set(CMAKE_ASM_COMPILER  ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT})
set(CMAKE_LINKER        ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT})
set(CMAKE_SIZE_UTIL     ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}size${TOOLCHAIN_EXT})
set(CMAKE_OBJCOPY       ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}objcopy${TOOLCHAIN_EXT})
set(CMAKE_OBJDUMP       ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}objdump${TOOLCHAIN_EXT})
set(CMAKE_NM_UTIL       ${ARM_TOOLCHAIN_PATH}/${TARGET_TRIPLET}nm${TOOLCHAIN_EXT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
#-lc -specs=nosys.specs
#set(CMAKE_COMMON_FLAGS "--specs=nano.specs -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums -Wall -Wextra -Wuninitialized -Wdouble-promotion -Werror -Wno-unused-parameter")
set(CMAKE_COMMON_FLAGS "--specs=nosys.specs -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums -Wall -Wextra -Wuninitialized -Wdouble-promotion -Werror -Wno-unused-parameter")

set(CMAKE_C_FLAGS 	"${MCPU_FLAGS} ${VFP_FLAGS} ${CMAKE_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${MCPU_FLAGS} ${VFP_FLAGS} ${CMAKE_COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS "${MCPU_FLAGS} ${VFP_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${LD_FLAGS} -fno-common -Wl,--gc-sections,-print-memory-usage")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_ASM_FLAGS_DEBUG "-g3")

set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_CXX_FLAGS_RELEASE "-Os")
set(CMAKE_ASM_FLAGS_RELEASE "")
