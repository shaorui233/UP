FILE(GLOB_RECURSE headers WB*.hpp)
FILE(GLOB_RECURSE sources WB*.cpp)


add_library(WBIC_Strict SHARED ${sources} ${headers})
target_link_libraries(WBIC_Strict Goldfarb_Optimizer)
