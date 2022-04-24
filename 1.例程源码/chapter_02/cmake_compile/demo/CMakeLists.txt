cmake_minimum_required (VERSION 2.8)
project(demo)

include_directories("${PROJECT_BINARY_DIR}")

add_library(foo foo.cpp)
add_executable (demo main.cpp)
target_link_libraries (demo foo)
