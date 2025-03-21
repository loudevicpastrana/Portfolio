set(TARGET adafruit_ili9341)

set(SOURCES
    Adafruit_ILI9341.cpp
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
)

target_link_libraries(${TARGET}
    adafruit_gfx
    drivers
)

# Disable warnings:
target_compile_options(${TARGET} PRIVATE -Wno-empty-body)
