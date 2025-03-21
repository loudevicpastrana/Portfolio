set(TARGET adafruit_vs1053)

set(SOURCES
    Adafruit_VS1053.cpp
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
        ../../Arduino/SD/src
)

target_link_libraries(${TARGET}
    drivers
)
