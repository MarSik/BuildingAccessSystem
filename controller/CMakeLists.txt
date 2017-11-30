# Generated from embedded cmake toolchain template 
cmake_minimum_required(VERSION 2.8)
PROJECT(project)

# Set used CPU and source list
SET(CPU cortex-m4)
SET(F_CPU 80000000)
SET(SOURCES src/DoorOpenerSketch.cpp src/Secrets.cpp src/Utils.cpp src/cardmanager.cpp)

# Precompile Arduino framework
FIND_PACKAGE(Arduino COMPONENTS tivac EK-TM4C123GXL)
ADD_LIBRARY(_arduino STATIC ${Arduino_SOURCES})
TARGET_INCLUDE_DIRECTORIES(_arduino PRIVATE ${Arduino_INCLUDE_DIRS})
TARGET_COMPILE_OPTIONS(_arduino PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-Xassembler -mccs> -Dgcc -DENERGIA_ARCH_TIVAC -DENERGIA_EK_TM4C123GXL -DARDUINO=10610 -DENERGIA=10002 -DF_CPU=${F_CPU})
SET_PROPERTY(TARGET _arduino APPEND_STRING PROPERTY LINK_FLAGS " -Wl,-Map=_arduino.map")
SET (Arduino_LINK_CMD -Wl,--whole-archive _arduino -Wl,--no-whole-archive)

# Use Arduino provided linker and startup script
SET("${PROJECT_NAME}_LINKER_SCRIPT" "${Arduino_LINKER_SCRIPT}")
SET(TOOLCHAIN_SOURCES "")

# Target name must match project name !!
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${Arduino_INCLUDE_DIRS} "src/include")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Arduino_LINK_CMD})
SET_PROPERTY(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS " -DENERGIA_ARCH_TIVAC -Dgcc -DENERGIA_EK_TM4C123GXL -DARDUINO=10610 -DENERGIA=10002")

USE(SPI ${PROJECT_NAME} ${Arduino_LIBRARY_DIR})
USE(Time ${PROJECT_NAME})
USE(Timezone ${PROJECT_NAME})
USE(SD ${PROJECT_NAME})
USE(DCF77 ${PROJECT_NAME})
USE(RC522 ${PROJECT_NAME})
USE(embdlog ${PROJECT_NAME})
USE(etl ${PROJECT_NAME})

# Project specific toolchain initialization
INCLUDE(${TOOLCHAIN_INIT})

# Additional defines and properties
