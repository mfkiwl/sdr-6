cmake_minimum_required(VERSION 2.8)
project(sdr)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")
set(CMAKE_CXX_FLAGS "-Wall -Wextra -g")

find_package(Eigen3 REQUIRED)

add_subdirectory(libraries/kalman-cpp/)
add_subdirectory(libraries/yaml-cpp/)

set(CMAKE_BINARY_DIR "bin/")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "lib/")
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

set(LIBRARY_OUTPUT_DIRECTORY "lib/")
set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include/
    libraries/*/include
    libraries/
)

add_library(detailed_exception.o src/detailed_exception.cpp)

add_library(pose.o src/pose.cpp)
target_link_libraries(pose.o detailed_exception.o m)

add_library(preprocessing.o src/preprocessing.cpp)
target_link_libraries(preprocessing.o detailed_exception.o pose.o yaml-cpp)

add_executable(sdr src/source.cpp)
target_link_libraries(sdr pose.o detailed_exception.o preprocessing.o)
