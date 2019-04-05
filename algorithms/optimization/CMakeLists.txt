cmake_minimum_required(VERSION 3.5)
project(optimization)

include_directories("include/")
include_directories("./")
include_directories("../../common/include/")
include_directories("../../third-party")
include_directories("../../third-party/include")
include_directories("../../third-party/ParamHandler")
#include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../robot)
include_directories(${CMAKE_BINARY_DIR}/third-party/nlopt/src/api)
include_directories(${CMAKE_BINARY_DIR})
file(GLOB sources "src/*.cpp" )

add_library(optimization SHARED ${sources})
add_executable(run_optimization main.cpp)
target_link_libraries(run_optimization optimization)

target_link_libraries(optimization biomimetics pthread)
target_link_libraries(optimization dynacore_param_handler)
target_link_libraries(optimization nlopt height_map)

add_subdirectory(height_map)
