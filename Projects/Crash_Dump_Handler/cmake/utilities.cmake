# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

function(post_build TARGET)
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        add_custom_target(${TARGET}.bin ALL 
            DEPENDS ${TARGET}
            COMMAND ${CMAKE_OBJCOPY} -Obinary ${TARGET}.elf ${TARGET}.bin
            COMMAND ${CMAKE_OBJCOPY} -Oihex ${TARGET}.elf ${TARGET}.hex)            
    else()
        message(FATAL_ERROR "Unknown CMAKE_C_COMPILER_ID ${CMAKE_C_COMPILER_ID}")
    endif()
endfunction()

function(set_target_linker TARGET LINKER_SCRIPT)
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        target_link_options(${TARGET} PRIVATE -T${LINKER_SCRIPT})
        target_link_options(${TARGET} PRIVATE -Wl,-Map=${TARGET}.map)
        set_target_properties(${TARGET} PROPERTIES SUFFIX ".elf") 
    else()
        message(FATAL_ERROR "Unknown CMAKE_C_COMPILER_ID ${CMAKE_C_COMPILER_ID}")
    endif()
endfunction()

function(add_map_file TARGET_NAME MAP_FILE_NAME)
    target_link_options(${TARGET_NAME} PRIVATE -Wl,-Map=${MAP_FILE_NAME})
endfunction()
