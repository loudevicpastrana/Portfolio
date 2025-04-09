set(TARGET mqtt-packet)

set(SOURCES
    ./src/MQTTConnectServer.c
    ./src/MQTTSubscribeServer.c
    ./src/MQTTSerializePublish.c
    ./src/MQTTDeserializePublish.c
    ./src/MQTTUnsubscribeClient.c
    ./src/MQTTUnsubscribeServer.c
    ./src/MQTTPacket.c
    ./src/MQTTSubscribeClient.c
    ./src/MQTTFormat.c
    ./src/MQTTConnectClient.c
)

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        ./src
)
