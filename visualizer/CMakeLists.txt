cmake_minimum_required(VERSION 3.10)

project(quadtree-pathfinding-visualizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(spdlog)
find_package(SDL2_ttf)
find_package(SDL2)
find_package(argparse)

# ----- library imgui ------
file(GLOB IMGUI_SOURCES 3rd-party/imgui/*.cpp)
add_library(imgui ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC "3rd-party/imgui")
target_include_directories(imgui PUBLIC ${SDL2_INCLUDE_DIRS})

# ----- library true_clearance_field ------
file(GLOB TRUE_CLEARANCE_FIELD_SOURCES ../3rd-party/true-clearance-field/*.cpp)
add_library(true_clearance_field ${TRUE_CLEARANCE_FIELD_SOURCES})
target_include_directories(true_clearance_field
    PUBLIC "../3rd-party/true-clearance-field")

# ----- library qdpf ------
file(GLOB QDPF_SOURCES ../qdpf.cpp ../internal/*.cpp)
add_library(qdpf ${QDPF_SOURCES})
target_link_libraries(qdpf true_clearance_field)
target_include_directories(qdpf PUBLIC "../3rd-party" "..")

# ----- quadtree-pathfinding-visualizer ------
add_executable(quadtree-pathfinding-visualizer main.cpp)
target_include_directories(quadtree-pathfinding-visualizer PUBLIC ".."
    "./3rd-party")

target_link_libraries(
    quadtree-pathfinding-visualizer
    qdpf
    imgui
    spdlog::spdlog
    sdl_ttf::sdl_ttf
    SDL2::SDL2main
    argparse::argparse)
