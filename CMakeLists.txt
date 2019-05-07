cmake_minimum_required (VERSION 2.8)
project (navx)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

FIND_PACKAGE (Threads REQUIRED)

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        #"${CMAKE_CURRENT_SOURCE_DIR}/include/ahrs/*.h"
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahrs
  #CATKIN_DEPENDS 
  #DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

list(REMOVE_ITEM SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TimeStamp.cpp
)

set(HEADER_FILES
  "include/ahrs/AHRS.h"
  "include/ahrs/AHRSProtocol.h"
  "include/ahrs/ContinuousAngleTracker.h"
  "include/ahrs/IBoardCapabilities.h"
  "include/ahrs/IIOCompleteNotification.h"
  "include/ahrs/IIOProvider.h"
  "include/ahrs/IMUProtocol.h"
  "include/ahrs/IMURegisters.h"
  "include/ahrs/InertialDataIntegrator.h"
  "include/ahrs/ITimestampedDataSubscriber.h"
  "include/ahrs/OffsetTracker.h"
  "include/ahrs/SerialIO.h"
  "include/ahrs/SerialPort.h")

add_library(ahrs SHARED ${SOURCES})
set_target_properties(ahrs PROPERTIES 
  PUBLIC_HEADER "${HEADER_FILES}"
  FRAMEWORK TRUE
)

#add_executable(NavXTimeStamp
#        "${CMAKE_CURRENT_SOURCE_DIR}/src/TimeStamp.cpp"
#)
#target_link_libraries(NavXTimeStamp ahrs ${CMAKE_THREAD_LIBS_INIT})
#target_compile_features(NavXTimeStamp PRIVATE cxx_range_for)

install(TARGETS ahrs 
        LIBRARY DESTINATION /usr/local/lib
        PUBLIC_HEADER DESTINATION /usr/local/include/ahrs
)

# To actually install headers
#install(FILES include/ahrs/AHRS.h
#  DESTINATION /usr/local/include/ahrs
#)
#
#install(TARGETS ahrs NavXTimeStamp
#        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
#        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
#        PUBLIC_HEADER DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/build
#)
