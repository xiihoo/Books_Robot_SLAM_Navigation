cmake_minimum_required(VERSION 2.8)
project(laser_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_DEFINITIONS(-std=c++11)


set(SDK_PATH "../sdk/")

IF (WIN32)
FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/windows/*.cpp"
  "${SDK_PATH}/src/*.cpp"
)

ELSE()

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
)

ENDIF()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDK_PATH}//include
  ${SDK_PATH}/src
)

set(MODULE_FILES
        laser_test.h
        laser_test.cpp
        )

add_executable(laser_test main.cpp ${MODULE_FILES} ${SDK_SRC})

IF (WIN32)
ELSE()
target_link_libraries(laser_test rt pthread)
ENDIF()
