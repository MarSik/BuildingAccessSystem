# Generated from embedded cmake toolchain template 
cmake_minimum_required(VERSION 2.8)
PROJECT(project)

# Set used CPU and source list
SET(CPU cortex-m4)
SET(F_CPU 80000000)
SET(SOURCES src/DoorOpenerSketch.cpp src/Secrets.cpp src/Utils.cpp src/cardmanager.cpp src/states.cpp src/countdown.cpp src/keyboard.cpp src/hal.cpp src/bluetooth.cpp src/ssp.c)

# Precompile Arduino framework
FIND_PACKAGE(Arduino COMPONENTS tivac EK-TM4C123GXL)
TARGET_COMPILE_OPTIONS(Arduino PRIVATE -fstack-protector-strong)

# Use Arduino provided linker and startup script
SET("${PROJECT_NAME}_LINKER_SCRIPT" "${Arduino_LINKER_SCRIPT}")
SET(TOOLCHAIN_SOURCES "")

# Target name must match project name !!
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${Arduino_INCLUDE_DIRS} "src/include")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Arduino_LIBRARIES})
TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE ${Arduino_COMPILE_FLAGS} -DF_CPU=${F_CPU} -DBOOST_NO_EXCEPTIONS -DBOOST_EXCEPTION_DISABLE -DBOOST_NO_RTTI -DBOOST_NO_TYPEID -fstack-protector-strong)

USE(SPI ${PROJECT_NAME} ${Arduino_LIBRARY_DIR})
USE(Time ${PROJECT_NAME})
USE(Timezone ${PROJECT_NAME})
USE(SD ${PROJECT_NAME})
USE(DCF77 ${PROJECT_NAME})
USE(RC522 ${PROJECT_NAME})
USE(embdlog ${PROJECT_NAME})
USE(etl ${PROJECT_NAME})
USE("tinyfsm-0.2.0" ${PROJECT_NAME})
USE(sha256 ${PROJECT_NAME})

# Project specific toolchain initialization
INCLUDE(${TOOLCHAIN_INIT})

# Additional defines and properties
