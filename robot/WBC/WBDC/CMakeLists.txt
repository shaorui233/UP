FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library(WBDC SHARED ${sources} ${headers})
target_link_libraries(WBDC Goldfarb_Optimizer)
