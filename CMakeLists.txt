# The MIT License (MIT)

# Copyright (c) 2014.4 JZ Xuan <jzxuanuni@gmail.com>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# CMakeLists for my research project.
project(slam)
cmake_minimum_required(VERSION 2.8)

# For test
add_definitions(-DPRINT_TRACE)

# short-hand functions
function(status line)
  message(STATUS ${line})
endfunction(status)

function(fatal line)
  message(FATAL ${line})
endfunction(fatal)

# Verbose make process
option(CMAKE_VERBOSE "Verbose Makefile" ON)
if(CMAKE_VERBOSE)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

# Build Options
option(BUILD_FRONT_END "build system front-end: robot control & rgb-d sensor" ON)
option(BUILD_BACK_END "build system back-end: vo trakcing & map reconstruct" ON)
option(BUILD_TEST "build unit test" ON)

# ---------------------------------------------------------------------------- #
#             Common settings for ROBOT and REMOTE systems
# ---------------------------------------------------------------------------- #
# output directories for project build
set(slam_RUNTIME_OUTPUT_DIRECTORY ${slam_SOURCE_DIR}/bin
    CACHE PATH "Output directories for binaries.")
set(slam_LIBRARY_OUTPUT_DIRECTORY ${slam_SOURCE_DIR}/lib
    CACHE PATH "Output directories for libraries.")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${slam_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${slam_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${slam_LIBRARY_OUTPUT_DIRECTORY})

# --------------------------- Find Common Packages --------------------------- #
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# Build common libraries
# TODO: write module to find OpenNI2
set(OpenNI2_INC_DIR "/usr/local/include/openni2/"
    CACHE PATH "path to openni2 headers.")
set(OpenNI2_LINK_DIR "/usr/local/lib/"
    CACHE PATH "path to openni2 libs.")
include_directories(${OpenNI2_INC_DIR})
link_directories(${OpenNI2_LINK_DIR})
set(OpenNI2_LIBS OpenNI2)

# Boost Library
# TODO: write a module to find boost
# Used macport to install (v1.55), path: /opt/local/
set(Boost_INC_DIR "/opt/local/include/"
    CACHE PATH "path to boost headers.")
set(Boost_LINK_DIR "/opt/local/lib/"
    CACHE PATH "path to boost libs.")
include_directories(${Boost_INC_DIR})
link_directories(${Boost_LINK_DIR})
list(APPEND Boost_LIBS
     boost_log-mt boost_log_setup-mt
     boost_system-mt boost_filesystem-mt boost_thread-mt)

# Gtest Library
if(BUILD_TEST)

  # TODO: write a module to find gtest
  find_path(Gtest_INC_DIR gtest)
  find_library(Gtest_main gtest_main)
  find_library(Gtest_test gtest)

  if(Gtest_INC_DIR)
    include_directories(${Gtest_INC_DIR})
    status("Found gtest include dir: ${Gtest_INC_DIR}.")
  else(Gtest_INC_DIR)
    fatal("Cannot found gtest include idr.")
  endif(Gtest_INC_DIR)

  if(Gtest_main)
    status("Found lib: ${Gtest_main}.")
    list(APPEND Gtest_LIBS ${Gtest_main})
  else(Gtest_main)
    fatal("Cannot found gtest_main lib.")
  endif(Gtest_main)

  if(Gtest_test)
    status("Found lib: ${Gtest_test}.")
    list(APPEND Gtest_LIBS ${Gtest_test})
  else(Gtest_test)
    fatal("Cannot found gtest_test lib.")
  endif(Gtest_test)

  if(UNIX)
    list(APPEND Gtest_test pthread)
    list(APPEND Gtest_main pthread)
    list(APPEND Gtest_LIBS pthread)
  endif(UNIX)

endif(BUILD_TEST)

# --------------------------- Compiler Related ------------------------------- #
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type: Release/Debug" FORCE)
endif(NOT CMAKE_BUILD_TYPE)
if (APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DBOOST_LOG_DYN_LINK"
    CACHE STRING "build flags for both debug and release" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-mmmx -msse -msse2 -msse3 -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-NDEBUG")
# Compier Flags = CMAKE_CXX_FLAGS + CMAKE_CXX_FLAGS_RELEASE/DEBUG
status("")
status("  Platform: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_VERSION}")
status("")
status("  Build Type: ${CMAKE_BUILD_TYPE}")
status("")
status("  C/C++")
status("    C++ Compiler:            ${CMAKE_CXX_FLAGS}")
status("    C++ flags (Release):     ${CMAKE_CXX_FLAGS_RELEASE}")
status("    C++ flags (Debug):       ${CMAKE_CXX_FLAGS_DEBUG}")
status("")
# ------------------------------- xnix related ------------------------------- #
add_library(xnix xnix_utility.c message.cpp)
# ---------------------------------------------------------------------------- #
#                             System Front-end Only
# ---------------------------------------------------------------------------- #
if (BUILD_FRONT_END)
  add_library(front_end irobot.cpp)
endif (BUILD_FRONT_END)
# ---------------------------------------------------------------------------- #
#                             System Back-end Only
# ---------------------------------------------------------------------------- #
if(BUILD_BACK_END)
  find_package(OPENGL REQUIRED)
  find_package(GLUT REQUIRED)

  # include g2o
  include_directories(${slam_SOURCE_DIR}/g2o-c++11/)
  link_directories(${slam_SOURCE_DIR}g2o-c++11/lib)

  # TODO: write module to find eigen3 packages
  find_path(Eigen_INCLUDE_DIRS eigen3)
  if(Eigen_INCLUDE_DIRS)
    set(Eigen_INCLUDE_DIRS "${Eigen_INCLUDE_DIRS}/eigen3")
    include_directories(Eigen_INCLUDE_DIRS "${Eigen_INCLUDE_DIRS}")
    message(STATUS "Foud Eigen: ${Eigen_INCLUDE_DIRS}")
  else()
    message(FATAL_ERROR "Cannot find gtest libgtest.a")
  endif()

  add_subdirectory(${slam_SOURCE_DIR}/rgbdvo)
  include_directories(${slam_SOURCE_DIR}/)
  link_directories(${slam_SOURCE_DIR}/rgbdvo/lib)

  set(G2O_LIBS "")
  list(APPEND G2O_LIBS
       g2o_cli g2o_stuff g2o_core g2o_solver_cholmod g2o_solver_csparse
       g2o_solver_dense g2o_solver_eigen g2o_solver_pcg g2o_solver_slam2d_linear
       g2o_solver_structure_only g2o_types_slam3d)

  # add all libraries
  list(APPEND BACK_END_DEP_LIBS
       rgbdvo pthread ${OpenNI2_LIBS}
       ${OPENGL_LIBRARY} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBS}
       ${G2O_LIBS})

  add_library(back_end sensor.cpp gl.cpp ext_utility.cpp helper.cpp
              pose_graph.cpp)

  add_executable(demo hand_held_demo.cpp)
  target_link_libraries(demo back_end ${BACK_END_DEP_LIBS})

  if(BUILD_TEST)
    add_executable(pose_graph_unit_test pose_graph_unit_test.cpp)
    target_link_libraries(pose_graph_unit_test back_end ${BACK_END_DEP_LIBS} ${Gtest_test})
    add_executable(message_test message_test.cpp)
    target_link_libraries(message_test xnix)
  endif(BUILD_TEST)
endif(BUILD_BACK_END)
