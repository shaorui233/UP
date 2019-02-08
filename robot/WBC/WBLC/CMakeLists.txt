FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBLC SHARED ${sources} ${headers})
#add_library(WBLC STATIC ${sources} ${headers})
target_link_libraries(WBLC Goldfarb_Optimizer)

#install(TARGETS WBLC DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/WBLC")
