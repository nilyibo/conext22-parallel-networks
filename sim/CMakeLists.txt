cmake_minimum_required(VERSION 3.10)
project(parallel_network_simulator CXX)

#set(CC_FLAGS "-Wpedantic -Wconversion -Wall")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CC_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CC_FLAGS}")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

message(STATUS "Build = ${CMAKE_BUILD_TYPE}")

add_subdirectory(pnet)
add_subdirectory(htsim)
add_subdirectory(abseil-cpp)

#add_dependencies(pnet_sim htsim)
#add_dependencies(parse_output htsim)
