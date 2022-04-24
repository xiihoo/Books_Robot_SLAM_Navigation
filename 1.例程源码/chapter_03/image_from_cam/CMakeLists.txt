cmake_minimum_required(VERSION 2.8)
project(image_from_cam)

find_package(OpenCV REQUIRED)
add_executable(image_from_cam image_from_cam.cpp)
target_link_libraries(image_from_cam ${OpenCV_LIBS})
