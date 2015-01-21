cmake_minimum_required(VERSION 2.6)

FILE(GLOB serial_src src/*.cpp)
FILE(GLOB serial_test_src test/*.cpp)
INCLUDE_DIRECTORIES(src)
#-DDEBUG
ADD_DEFINITIONS(-O2 -g -DDEBUG)
ADD_EXECUTABLE(test_serial test/test_serial.cpp)
ADD_EXECUTABLE(echo_serial test/echo_serial.cpp)

ADD_LIBRARY(common_serial SHARED ${serial_src})

            
TARGET_LINK_LIBRARIES(test_serial common_serial pthread )
TARGET_LINK_LIBRARIES(echo_serial common_serial pthread )

INSTALL_TARGETS(/bin test_serial)
INSTALL_TARGETS(/bin echo_serial)

INSTALL_TARGETS(/lib  common_serial)
INSTALL_TARGETS(/bin  test_serial)
INSTALL_TARGETS(/bin  echo_serial)

