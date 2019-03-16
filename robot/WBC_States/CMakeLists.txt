FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

set(STATE_NAME WBC)

include_directories("../../algorithms/optimization/include")
include_directories("../../algorithms/optimization")
include_directories("./../")

add_library (${STATE_NAME}_state SHARED ${headers} ${sources} )
target_link_libraries (${STATE_NAME}_state biomimetics)
target_link_libraries (${STATE_NAME}_state WBLC dynacore_param_handler)

