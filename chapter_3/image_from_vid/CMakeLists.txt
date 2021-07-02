cmake_minimum_required(VERSION 2.8)
project(image_from_vid)

find_package(OpenCV REQUIRED)
add_executable(image_from_vid image_from_vid.cpp)
target_link_libraries(image_from_vid ${OpenCV_LIBS})
