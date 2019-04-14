cmake_minimum_required(VERSION 3.5)
project(machine_learning)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
 -Wsign-promo -Werror")
 

set(CMAKE_COLOR_MAKEFILE   ON)
message("LCM machine learning")
execute_process(COMMAND ../algorithms/machine_learning/scripts/lcm_generation.sh)
execute_process(COMMAND ../algorithms/machine_learning/scripts/swig_wrap_generation.sh)

include_directories("./include")
include_directories("./bridge_lcm_types")
include_directories("./../../common/include/")
include_directories("./../../robot/WBC_States/")
include_directories(${CMAKE_BINARY_DIR})

# Python package
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

file(GLOB_RECURSE swig_output_cpp "python_bridge/*.cpp")

add_library(pycheetah SHARED python_bridge/python_bridge.cpp ${swig_output_cpp})

target_link_libraries(pycheetah dynacore_param_handler lcm)
target_link_libraries(pycheetah ${PYTHON_LIBRARIES})
set_target_properties(pycheetah PROPERTIES PREFIX "" OUTPUT_NAME "_pycheetah")


########## Cheetah Simulation #########################
set(CMAKE_PREFIX_PATH ~/Qt/5.10.0/gcc_64)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

set(Qt5Core_DIR /Qt/5.10.0/gcc_64/lib/cmake/Qt5Core)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
include_directories(${OPENGL_INCLUDE_DIR})
set(QOBJECT_HEADERS
    include/SimGraphics3D.hpp)
 
QT5_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})
#message(${tst_hdr_moc})
file(GLOB_RECURSE cheetah_cpp "src/*.cpp")
#message(${cheetah_cpp})
add_executable(CheetahSim ${cheetah_cpp} ${tst_hdr_moc})
target_link_libraries(CheetahSim lcm biomimetics pthread lcm
    dynacore_param_handler Qt5::Widgets Qt5::Core Qt5::Gui ${QT_LIBRARIES}
    ${OPENGL_LIBRARIES} glut)
