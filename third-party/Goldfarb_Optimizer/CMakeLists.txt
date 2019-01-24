FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)
add_library (Goldfarb_Optimizer SHARED ${headers} ${sources})

#install(TARGETS Goldfarb_Optimizer DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/Optimizer/Goldfarb_Optimizer")
