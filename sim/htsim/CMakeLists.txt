file(GLOB htsim_SRC *.cpp)
list(REMOVE_ITEM htsim_SRC parse_output.cpp main.cpp)

add_library(htsim ${htsim_SRC})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(parse_output_SRC
	parse_output.cpp
)

add_executable(parse_output ${parse_output_SRC})
target_link_libraries(parse_output htsim)
