# CMakeLists.txt
cmake_minimum_required(VERSION 3.0)
project(Kalman)
message(STATUS "The BINARY directory " ${Kalman_BINARY_DIR})
message(STATUS "The BINARY directory " ${Kalman_BINARY_DIR})
set(SRC_LIST main.c kalman_filter.c)

# Define executable target
include_directories(${PROJECET_SOURCE_DIR})
add_executable(Kalman_Result ${SRC_LIST})