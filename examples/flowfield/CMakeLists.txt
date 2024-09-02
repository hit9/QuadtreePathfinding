cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-example-flowfield)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# ----- library true_clearance_field ------
file(GLOB TRUE_CLEARANCE_FIELD_SOURCES
    ../../3rd-party/true-clearance-field/*.cpp)
add_library(true_clearance_field ${TRUE_CLEARANCE_FIELD_SOURCES})
target_include_directories(true_clearance_field
    PUBLIC "../../3rd-party/true-clearance-field")

# ----- library qdpf ------
file(GLOB QDPF_SOURCES ../../qdpf.cpp ../../internal/*.cpp)
add_library(qdpf ${QDPF_SOURCES})
target_include_directories(qdpf PUBLIC "../../3rd-party" "../..")
target_link_libraries(qdpf true_clearance_field)

# ----- executable quadtree-pathfinding-example-flowfield ------
add_executable(quadtree-pathfinding-example-flowfield main.cpp)
target_include_directories(quadtree-pathfinding-example-flowfield
    PUBLIC "../..")
target_link_libraries(quadtree-pathfinding-example-flowfield qdpf)
