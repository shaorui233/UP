cmake_minimum_required(VERSION 3.5)
project(sim)
set(CMAKE_PREFIX_PATH ~/Qt/5.10.0/gcc_64)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -g -march=native") # set here
set(CMAKE_C_FLAGS "-O3  -g  -march=native -std=gnu99 -I.")
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

set(Qt5Core_DIR /Qt/5.10.0/gcc_64/lib/cmake/Qt5Core)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
find_package(Qt5Gui CONFIG REQUIRED)

#set(QOBJECT_HEADERS
#        include/main_window.h
#        include/gamepad_monitor.h
#        include/spi_debug.h)

QT5_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})



include_directories("include/")
include_directories("../common/include/")
include_directories("../third-party/include")
include_directories("../lcm-types")
file(GLOB sources "src/*.cpp" "src/rt/*.c" "../lcm-types/c_lcm_files/*.c" "../lcm-types/c_lcm_files/*.h")

find_package(Qt5Core COMPONENTS QtGamepad REQUIRED)
find_package(Qt5Gamepad REQUIRED)



add_executable(sim ${sources} ${tst_hdr_moc})
target_link_libraries(sim biomimetics pthread lcm Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Gamepad ${QT_LIBRARIES})