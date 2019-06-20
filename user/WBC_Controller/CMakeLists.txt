cmake_minimum_required(VERSION 3.5)
project(WBC_Controller)

include_directories("./")
include_directories("../../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories(SYSTEM "../third-party/qpOASES/include/")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(WBC_State)
add_subdirectory(WBC)
add_executable(WBC_Controller main.cpp WBC_Controller.cpp)
target_link_libraries(WBC_Controller robot biomimetics)
target_link_libraries(WBC_Controller WBC_State)

