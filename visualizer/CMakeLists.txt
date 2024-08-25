cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-visualizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include_directories("..")
include_directories("../3rd-party")
file(GLOB QDPF_SOURCES ../qdpf.cpp ../internal/*.cpp)
add_library(qdpf ${QDPF_SOURCES})
add_executable(quadtree-pathfinding-visualizer main.cpp)

find_package(SDL2)
find_package(spdlog)
find_package(argparse)

target_link_libraries(quadtree-pathfinding-visualizer qdpf SDL2::SDL2main
                      spdlog::spdlog argparse::argparse)
