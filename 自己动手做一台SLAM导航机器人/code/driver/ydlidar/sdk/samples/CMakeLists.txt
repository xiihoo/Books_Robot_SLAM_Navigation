
cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

set(YDLIDAR_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
IF (WIN32)
list(APPEND YDLIDAR_LIBRARIES  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/libydlidar_driver.dll)
ELSE()
list(APPEND YDLIDAR_LIBRARIES  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/libydlidar_driver.so)
ENDIF()


#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
    ${YDLIDAR_INCLUDE_DIRS}
)


ADD_EXECUTABLE(${PROJECT_NAME}
               main.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${YDLIDAR_LIBRARIES})
