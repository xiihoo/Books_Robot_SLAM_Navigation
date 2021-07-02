cmake_minimum_required(VERSION 2.8)
project(xfeatures2d_example)

find_package(OpenCV REQUIRED)

#create sift executable
add_executable(sift_extract sift_extract.cpp)
target_link_libraries(sift_extract ${OpenCV_LIBS})
#create surf executable
add_executable(surf_extract surf_extract.cpp)
target_link_libraries(surf_extract ${OpenCV_LIBS})
#create orb executable
add_executable(orb_extract orb_extract.cpp)
target_link_libraries(orb_extract ${OpenCV_LIBS})
