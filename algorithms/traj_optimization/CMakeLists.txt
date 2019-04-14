cmake_minimum_required(VERSION 2.8.2)
project(traj_optimization)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wshadow -Wsign-promo")
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

# Source/Include files
include_directories(".")
include_directories("./../../common/include")
include_directories("./../../third-party/nlopt/")
include_directories("./../../third-party/snopt/include")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_BINARY_DIR}/third-party/nlopt/src/api)

file(GLOB_RECURSE test_sources "gtest/*.cpp")             # test cpp files

include(CTest)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Test
add_executable(test_trj_opt ${test_sources})
target_link_libraries(test_trj_opt gtest gmock_main)
target_link_libraries(test_trj_opt Goldfarb_Optimizer nlopt snopt)

find_library(snopt7_c++ NAMES libsnopt7_cpp.so PATHS ./../../third-party/snopt/lib/ REQUIRED)
target_link_libraries(test_trj_opt ${snopt7_c++})
