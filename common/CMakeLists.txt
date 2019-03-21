cmake_minimum_required(VERSION 2.8.2)

project(libbiomimetics)

# Compiler flags
set(CMAKE_CXX_FLAGS "-O3 -g -march=native -ffast-math -ggdb -Wall -Wall \
-Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wredundant-decls -Wshadow -Wsign-promo -Werror")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -I /usr/include/")
set(CMAKE_C_FLAGS "-O3  -g -std=gnu99 -march=native -I.")

# Pull in Google Test
include(CTest)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()
include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()


# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../lcm-types/cpp/")
include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("../third-party/nlopt/")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_BINARY_DIR}/third-party/nlopt/src/api)

file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files
file(GLOB_RECURSE test_sources "test/*.cpp")             # test cpp files

# Library
#add_library(biomimetics STATIC ${sources})       # produce a library used by sim/robot
add_library(biomimetics SHARED ${sources})       # produce a library used by sim/robot
target_link_libraries(biomimetics inih dynacore_param_handler)

# Test
add_executable(test-common ${test_sources} ${sources})
target_link_libraries(test-common gtest gmock_main lcm rt inih osqp dynacore_param_handler)
target_link_libraries(test-common Goldfarb_Optimizer nlopt)
target_link_libraries(test-common WBC_state)
add_test(NAME example_test COMMAND test-common)
