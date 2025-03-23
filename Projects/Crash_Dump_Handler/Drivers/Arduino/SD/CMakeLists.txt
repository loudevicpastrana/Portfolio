set(TARGET arduinosd)

set(SOURCES
    ./src/File.cpp
    ./src/SD.cpp
    ./src/utility/SdVolume.cpp
    ./src/utility/SdFile.cpp
    ./src/utility/Sd2Card.cpp
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        ./src
)

target_link_libraries(${TARGET}
    drivers
)

# Disable warnings:
target_compile_options(${TARGET} PRIVATE -Wno-address-of-packed-member)
