FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

set(STATE_NAME WBC)

add_library (${STATE_NAME}_state SHARED ${headers} ${sources} )
#add_library (${STATE_NAME}_state STATIC ${headers} ${sources} )
target_link_libraries (${STATE_NAME}_state biomimetics)
target_link_libraries (${STATE_NAME}_state WBLC)
target_link_libraries (${STATE_NAME}_state dynacore_param_handler)

#install(TARGETS ${STATE_NAME}_state DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/${STATE_NAME}_state")
