cmake_minimum_required(VERSION 2.8)
project(calc_hist)

find_package(OpenCV REQUIRED)
add_executable(calc_hist calc_hist.cpp)
target_link_libraries(calc_hist ${OpenCV_LIBS})
