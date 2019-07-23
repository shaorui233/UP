FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)


add_library (WBC_Ctrl SHARED ${headers} ${sources} )
target_link_libraries (WBC_Ctrl biomimetics lcm)
target_link_libraries (WBC_Ctrl WBIC WBLC dynacore_param_handler)

if(IPOPT_OPTION)
link_directories("../../../../third-party/CoinIpopt/build/lib")
include_directories(SYSTEM "../../../../third-party/CoinIpopt/build/include/coin")
find_library(ipopt NAMES libipopt.so PATHS "../../../../third-party/CoinIpopt/build/lib")
target_link_libraries(WBC_Ctrl ${ipopt})
endif(IPOPT_OPTION)


