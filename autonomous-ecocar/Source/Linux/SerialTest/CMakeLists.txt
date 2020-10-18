cmake_minimum_required(VERSION 3.5.1)
project(SerialTest)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp serial_tools.h)
add_executable(SerialTest ${SOURCE_FILES})

link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

target_link_libraries(SerialTest serialport)
