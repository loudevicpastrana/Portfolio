set(TARGET es_wifi)

set(SOURCES
    es_wifi.c
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
)

target_link_libraries(${TARGET}
    drivers
    cmsis_os_ucos2
)
