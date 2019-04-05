cmake_minimum_required(VERSION 2.8.2)
project(libpath_planning)

# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("../../third-party/ParamHandler")
include_directories("../../common/include/")
include_directories(${CMAKE_BINARY_DIR})

file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files

# Library
add_library(path_planning SHARED ${sources})       # produce a library used by sim/robot

add_executable(path_planning_test test_main.cpp)
target_link_libraries(path_planning_test path_planning biomimetics pthread)
