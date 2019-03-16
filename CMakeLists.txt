cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

add_definitions(-O3)
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
execute_process(COMMAND ../scripts/make_types.sh)

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake 
    ${CMAKE_BINARY_DIR}/Configuration.h)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)
add_subdirectory(sim)
#add_subdirectory(algorithms)
