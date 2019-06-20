include_directories("../../robot/include")
include_directories("../../common/include/")

add_executable(jpos_ctrl main.cpp JPos_Controller.cpp)
target_link_libraries(jpos_ctrl robot)
