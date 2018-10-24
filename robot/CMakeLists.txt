cmake_minimum_required(VERSION 3.5)
project(robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -g -march=native") # set here
set(CMAKE_C_FLAGS "-O3  -g  -march=native -std=gnu99 -I.")

include_directories("include/")
include_directories("../common/include/")
include_directories("../third-party/include")
include_directories("../lcm-types/c_lcm_files")
file(GLOB sources "src/*.cpp" "src/rt/*.c" "../lcm-types/c_lcm_files/*.c" "../lcm-types/c_lcm_files/*.h")

add_executable(robot ${sources})
target_link_libraries(robot biomimetics pthread lcm ${CMAKE_SOURCE_DIR}/third-party/libsoem.a ${CMAKE_SOURCE_DIR}/third-party/libvnc.a)