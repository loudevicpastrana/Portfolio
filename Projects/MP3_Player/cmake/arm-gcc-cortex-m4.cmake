# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

# Original at: https://github.com/azure-rtos/getting-started/


# See more info about ARM flags at:
# https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html 
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/architecture-support
# https://stackoverflow.com/questions/16768235/cortex-m4-gcc-float-behavior

set(MCPU_FLAGS "-mthumb -mcpu=cortex-m4 -DUSE_FULL_LL_DRIVER -DSTM32L475xx -DUSE_HAL_DRIVER")
set(VFP_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")

include(${CMAKE_CURRENT_LIST_DIR}/arm-gcc-cortex-toolchain.cmake)
