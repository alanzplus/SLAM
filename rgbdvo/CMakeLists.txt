# CMakeLists for rgbvo
project(rgbdvo)
cmake_minimum_required(VERSION 2.6)

# Set the output directory for rgbdvo
set(rgbdvo_LIBRARY_OUTPUT_DIRECTORY ${rgbdvo_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${rgbdvo_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${rgbdvo_LIBRARY_OUTPUT_DIRECTORY})

# Build static library, that is .a file
add_library(rgbdvo
    vo.cpp
    feature.cpp
    camera.cpp
    helper.cpp
    pyramid.cpp
    depth.cpp
    frame.cpp
    motion_estimate.cpp
    fm.cpp)
