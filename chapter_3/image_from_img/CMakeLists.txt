cmake_minimum_required(VERSION 2.8)
project(image_from_img)

find_package(OpenCV REQUIRED)
add_executable(image_from_img image_from_img.cpp)
target_link_libraries(image_from_img ${OpenCV_LIBS})
