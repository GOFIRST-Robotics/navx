cmake_minimum_required(VERSION 2.8.3)
project(navx)
set(CMAKE_BUILD_TYPE Release)

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        #"${CMAKE_CURRENT_SOURCE_DIR}/include/ahrs/*.h"
)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)
find_package (Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navx
  CATKIN_DEPENDS sensor_msgs roscpp
#  DEPENDS system_lib
#  DEPENDS Threads
)

###########
## Build ##
###########

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        #"${CMAKE_CURRENT_SOURCE_DIR}/include/ahrs/*.h"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(navx
 ${SOURCES}
)
add_dependencies(navx ${navx_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navx_node src/navx_node.cpp)
add_dependencies(navx_node ${navx_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navx_node
   ${catkin_LIBRARIES}
   navx
 )
