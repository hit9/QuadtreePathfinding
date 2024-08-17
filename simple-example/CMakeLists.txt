cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-example)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

file(GLOB SOURCES main.cpp ../quadtree_pathfinding.cpp)
add_executable(quadtree-pathfinding-example ${SOURCES})

include_directories("..")
include_directories("../3rd-party")
