cmake_minimum_required(VERSION 3.5)
project(robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -no-pie -g -march=native -ggdb -Wall -Wall \
-Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wredundant-decls -Wshadow -Wsign-promo -Werror")
set(CMAKE_C_FLAGS "-O3  -no-pie -g  -march=native -std=gnu99 -I.")

set(CMAKE_CXX_FLAGS "-Wno-redeclaration")
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/include")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories(${CMAKE_BINARY_DIR})
file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp" )

add_executable(robot ${sources})
target_link_libraries(robot biomimetics pthread lcm rt inih 
    libvnc
    )

add_subdirectory(WBC_States)
add_subdirectory(WBC)

target_link_libraries(robot dynacore_param_handler WBC_state)

