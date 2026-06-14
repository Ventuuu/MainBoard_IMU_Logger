cmake_minimum_required(VERSION 3.22)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MainBoard_IMU_Logger)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
add_compile_options(
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

add_link_options(
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    "LINKER:-T,${CMAKE_SOURCE_DIR}/STM32U575RITXQ_FLASH.ld"
)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/as7341_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/light_metrics_mcu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/bluetooth.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/imu_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/led_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Memory_operations.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SPI_NAND.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # Add user defined sources
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/App
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Device/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
