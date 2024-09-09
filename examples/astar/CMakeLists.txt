cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-example-astar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# ----- library clearance_field ------
file(GLOB CLEARANCE_FIELD_SOURCES ../../3rd-party/clearance-field/*.cpp)
add_library(clearance_field ${CLEARANCE_FIELD_SOURCES})
target_include_directories(clearance_field
                           PUBLIC "../../3rd-party/clearance-field")

# ----- library qdpf ------
file(GLOB QDPF_SOURCES ../../qdpf.cpp ../../internal/*.cpp)
add_library(qdpf ${QDPF_SOURCES})
target_include_directories(qdpf PUBLIC "../../3rd-party" "../..")
target_link_libraries(qdpf clearance_field)

# ----- executable quadtree-pathfinding-example-astar ------
add_executable(quadtree-pathfinding-example-astar main.cpp)
target_include_directories(quadtree-pathfinding-example-astar PUBLIC "../..")
target_link_libraries(quadtree-pathfinding-example-astar qdpf)
