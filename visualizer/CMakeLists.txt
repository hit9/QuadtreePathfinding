cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfiding-visualizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

file(GLOB SOURCES main.cpp ../quadtree_pathfinding.cpp)
add_executable(quadtree-pathfiding-visualizer ${SOURCES})

find_package(SDL2)
find_package(spdlog)
find_package(argparse)

include_directories("..")
include_directories("../3rd-party")

target_link_libraries(quadtree-pathfiding-visualizer SDL2::SDL2main
    spdlog::spdlog argparse::argparse)
