cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-example)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include_directories("../3rd-party")
include_directories("..")

file(GLOB QDPF_SOURCES ../qdpf.cpp ../internal/*.cpp)
add_library(qdpf ${QDPF_SOURCES})
add_executable(quadtree-pathfinding-example main.cpp)
target_link_libraries(quadtree-pathfinding-example qdpf)
