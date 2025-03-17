file(GLOB APP_LIB_SOURCE
     "crc.c"
     "sensor.c"
)

# Create a libary for the application sources
add_library(${APP_LIB_NAME} ${APP_LIB_SOURCE})
