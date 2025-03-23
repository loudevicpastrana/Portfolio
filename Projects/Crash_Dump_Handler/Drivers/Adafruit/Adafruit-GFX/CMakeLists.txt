set(TARGET adafruit_gfx)

set(SOURCES
    Adafruit_GFX.cpp
    glcdfont.c
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        .
)

# Disable warnings:
target_compile_options(${TARGET} PRIVATE -Wno-unused-const-variable -Wno-unknown-pragmas)