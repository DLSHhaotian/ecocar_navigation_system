cmake_minimum_required(VERSION 3.5)
project(MyQueue)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES my_queue.cpp my_queue.h)
add_library(MyQueue SHARED ${SOURCE_FILES})

INSTALL(TARGETS MyQueue
        DESTINATION /usr/local/lib)
INSTALL(FILES my_queue.h
        DESTINATION /usr/local/include)