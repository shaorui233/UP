project(footplan-viewer)

set(CMAKE_PREFIX_PATH ~/Qt/5.12.3/gcc_64)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

set(Qt5Core_DIR ~/Qt/5.10.0/gcc_64/lib/cmake/Qt5Core)
set(Qt5Widgets_DIR ~/Qt/5.10.0/gcc_64/lib/cmake/Qt5Widgets)
set(Qt5Gamepad_DIR ~/Qt/5.10.0/gcc_64/lib/cmake/Qt5Gamepad)

find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
find_package(Qt5Gui CONFIG REQUIRED)

set(QOBJECT_HEADERS
        FootplanWindow.h)

QT5_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})

include_directories("../common/FootstepPlanner")
include_directories("../common/include")

file(GLOB sources "./*.cpp")

find_package(Qt5Core COMPONENTS QtGamepad REQUIRED)
find_package(Qt5Gamepad REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

add_executable(footplan-viewer ${sources} ${tst_hdr_moc})

target_link_libraries(footplan-viewer footstep_planner pthread Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Gamepad ${QT_LIBRARIES} )