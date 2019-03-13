cmake_minimum_required(VERSION 3.5)
project(machine_learning)


set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-O3 -g -march=native -ggdb -Wall -Wall \
#-Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 \
#-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
#-Wredundant-decls -Wshadow -Wsign-promo -Werror")
#set(CMAKE_C_FLAGS "-O3  -g  -march=native -std=gnu99 -I.")


set(CMAKE_COLOR_MAKEFILE   ON)
message("LCM machine learning")
execute_process(COMMAND ../machine_learning/scripts/lcm_generation.sh)
execute_process(COMMAND ../machine_learning/scripts/swig_wrap_generation.sh)

include_directories("./include")
include_directories("./bridge_lcm_types")
include_directories("./../common/include/")
include_directories("./../robot/FSM_States/")
include_directories("./../lcm-types/")
include_directories(${CMAKE_BINARY_DIR})

# Python package
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

file(GLOB_RECURSE swig_output_cpp "python_bridge/*.cpp")
file(GLOB_RECURSE cheetah_cpp "src/*.cpp")

add_library(pycheetah SHARED python_bridge/python_bridge.cpp ${swig_output_cpp})

target_link_libraries(pycheetah biomimetics dynacore_param_handler lcm)
target_link_libraries(pycheetah ${PYTHON_LIBRARIES})
set_target_properties(pycheetah PROPERTIES PREFIX "" OUTPUT_NAME "_pycheetah")


add_executable(CheetahSim ${cheetah_cpp})
target_link_libraries(CheetahSim lcm)
