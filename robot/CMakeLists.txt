cmake_minimum_required(VERSION 3.5)
project(robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -g -march=native") # set here
set(CMAKE_C_FLAGS "-O3  -g  -march=native -std=gnu99 -I.")

include_directories("include/" )
include_directories("../common/include/")
file(GLOB sources "src/*.cpp")

add_executable(robot ${sources})
target_link_libraries(robot biomimetics)