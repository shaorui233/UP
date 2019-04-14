FILE(GLOB_RECURSE c_sources src/*.c)
FILE(GLOB_RECURSE cpp_sources src/*.cpp)
add_library (snopt SHARED ${c_sources} ${cpp_sources})
target_include_directories(snopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

#find_library(snopt7_c++ NAMES libsnopt7_cpp.so.0 libsnopt7_cpp.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
#find_library(snopt7_int NAMES libsnopt7.so.0 libsnopt7.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
#find_library(gfortran NAMES libgfortran.so.4 PATHS /usr/lib/x86_64-linux-gnu/)
   find_library(gfortran NAMES libgfortran.so.4 PATHS /usr/lib/x86_64-linux-gnu/)
find_library(snopt7_c++ NAMES libsnopt7_cpp.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
#find_library(snopt7_int NAMES libsnopt7.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)

#message(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (snopt ${snopt7_c++} ${gfortran})
#target_link_libraries (snopt ${snopt7_int} ) 

#add_definition(SNOPT_LICENSE=${CMAKE_CURRENT_SOURCE_DIR}/snopt7.lic)
