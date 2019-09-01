#include_directories(SYSTEM "../../../../third-party/qpOASES/include")
#include_directories("../../../../lcm-types/cpp")
#include_directories("/usr/local/include/lcm/")   # lcm includes
#include_directories("/usr/local/include/eigen3")   # lcm includes

FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (VisionMPC SHARED ${headers} ${sources} )
target_link_libraries (VisionMPC biomimetics lcm)

