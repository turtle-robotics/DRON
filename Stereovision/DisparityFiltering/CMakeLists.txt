cmake_minimum_required(VERSION 3.0)
project(DisparityFiltering)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(DisparityFiltering DisparityFiltering.cpp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY bin)
set(CMAKE_BUILD_TYPE Debug)
add_library(DisparityFiltering SHARED ../DisparityFiltering.cpp)

target_link_libraries(DisparityFiltering ${OpenCV_LIBS})