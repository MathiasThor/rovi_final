cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ROVI_FINAL)

find_package(OpenCV 3.1.0 EXACT REQUIRED)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_BINARY_DIR ${PROJECT_BINARY_DIR}/bin)

set(HEADERS headers/color_detector.h)
set(HEADERS headers/corny_detector.h)
set(HEADERS headers/general_functions.h)
set(HEADERS headers/setup.h)

include_directories(header_files)
add_subdirectory(source_files)

set( CMAKE_CXX_FLAGS
    "	${CMAKE_CXX_FLAGS} \
     -std=c++11 		      \
     -ffunction-sections 	 \
     -fdata-sections		    \
     -Wcast-qual		         \
     -Wno-unused-variable 	  \
     -pedantic		             \
     -g                         \
     -Ofast                      \
    " )
