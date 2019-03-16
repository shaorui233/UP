cmake_minimum_required(VERSION 2.8.2)
project(libpath_planning)

# Compiler flags
set(CMAKE_CXX_FLAGS "-O3 -g -march=native -ffast-math -ggdb -Wall -Wall \
-Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wredundant-decls -Wshadow -Wsign-promo -Werror")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -I /usr/include/")
set(CMAKE_C_FLAGS "-O3  -g -std=gnu99 -march=native -I.")

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
target_link_libraries(path_planning_test path_planning biomimetics)
