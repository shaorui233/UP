cmake_minimum_required(VERSION 3.5)
project(MIT_Controller)

include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories(SYSTEM "../third-party/qpOASES/include/")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories(${CMAKE_BINARY_DIR})
file(GLOB sources "FSM_States/*.h" "FSM_States/*.cpp"
"Controllers/BalanceController/*.cpp"
"Controllers/BalanceController/*.h"
"Controllers/BalanceController/*.hpp")

add_executable(MIT_Controller ${sources})
target_link_libraries(MIT_Controller robot biomimetics)
target_link_libraries(MIT_Controller dynacore_param_handler qpOASES)

