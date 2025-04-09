set(TARGET cmsis_os_ucos2)

set(SOURCES
    ./src/cmsis_os_ucos2.c
)

add_library(${TARGET}
    ${SOURCES}
)

target_link_libraries(${TARGET}
    ucos2
)

target_include_directories(${TARGET} 
    PUBLIC 
        ./src
)
