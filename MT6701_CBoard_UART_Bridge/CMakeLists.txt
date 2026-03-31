cmake_minimum_required(VERSION 3.22)

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME MT6701_CBoard_UART_Bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Users/Bsp/Bsp_Board/Board_CRC/Board_CRC8_CRC16.c
    Users/Bsp/Bsp_Device/Dev_MT6701/Dev_MT6701.c
    Users/Application/App_MT6701/App_MT6701_Task.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Users/Bsp/Bsp_Board/Board_CRC
    Users/Bsp/Bsp_Device/Dev_MT6701
    Users/Application/App_MT6701
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)