option(WITH_FIRST_FIT "Whether to use First Fit algorithm" OFF)
if(WITH_FIRST_FIT)
    add_compile_definitions(USE_FIRST_FIT=1)
else(WITH_FIRST_FIT)
    add_compile_definitions(USE_FIRST_FIT=0)
endif(WITH_FIRST_FIT)

option(WITH_SERVERS "Whether to include servers" ON)
if(WITH_SERVERS)
    add_compile_definitions(SIM_INCLUDE_SERVERS=1)
else(WITH_SERVERS)
    add_compile_definitions(SIM_INCLUDE_SERVERS=0)
endif(WITH_SERVERS)

option(WITH_LP_FAIR_SHARING "Whether to enable fair sharing on LP solver" OFF)
if(WITH_LP_FAIR_SHARING)
    add_compile_definitions(SIM_LP_FAIR_SHARING=1)
else(WITH_LP_FAIR_SHARING)
    add_compile_definitions(SIM_LP_FAIR_SHARING=0)
endif(WITH_LP_FAIR_SHARING)

option(WITH_CONSTANT_SEED "Whether to use constant seed" OFF)
if(WITH_CONSTANT_SEED)
    add_compile_definitions(USE_CONSTANT_SEED=1)
else(WITH_CONSTANT_SEED)
    add_compile_definitions(USE_CONSTANT_SEED=0)
endif(WITH_CONSTANT_SEED)

option(WITH_VERBOSE_LOGGING "Whether to turn on verbose logging" OFF)
if(WITH_VERBOSE_LOGGING)
    add_compile_definitions(LOG_LEVEL_VERBOSE=1)
else(WITH_VERBOSE_LOGGING)
    add_compile_definitions(LOG_LEVEL_VERBOSE=0)
endif(WITH_VERBOSE_LOGGING)

message(STATUS "WITH_FIRST_FIT = ${WITH_FIRST_FIT}")
message(STATUS "WITH_SERVERS = ${WITH_SERVERS}")
message(STATUS "WITH_LP_FAIR_SHARING = ${WITH_LP_FAIR_SHARING}")
message(STATUS "WITH_CONSTANT_SEED = ${WITH_CONSTANT_SEED}")
message(STATUS "WITH_VERBOSE_LOGGING = ${WITH_VERBOSE_LOGGING}")

if (APPLE AND EXISTS /usr/local/opt/libomp/lib/libomp.dylib)
    set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
    set(OpenMP_C_LIB_NAMES omp)
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
    set(OpenMP_CXX_LIB_NAMES omp)
    set(OpenMP_omp_LIBRARY /usr/local/opt/libomp/lib/libomp.dylib)
endif()

find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED serialization)

include_directories(
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/applications
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/hadoop
    ${CMAKE_CURRENT_SOURCE_DIR}/event_handlers
    ${CMAKE_CURRENT_SOURCE_DIR}/graph
    ${CMAKE_SOURCE_DIR}/htsim)
file(GLOB_RECURSE simulator_SRC "*.cc")

add_executable(pnet_sim ${simulator_SRC})
target_link_libraries(pnet_sim OpenMP::OpenMP_CXX htsim absl::str_format ${Boost_LIBRARIES})
