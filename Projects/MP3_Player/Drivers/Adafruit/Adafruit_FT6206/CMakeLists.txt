set(TARGET adafruit_ft6206)

set(SOURCES
    Adafruit_FT6206.cpp
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
)
