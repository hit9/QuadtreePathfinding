cmake_minimum_required(VERSION 3.10)

project(QuadtreePathfindingExampleAstar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# ----- library qdpf ------
file(GLOB_RECURSE QDPF_SOURCES ../../Source/*.cpp)
add_library(QDPF ${QDPF_SOURCES})
target_include_directories(QDPF PUBLIC ../../Source/3rdParty)

# ----- executable QuadtreePathfindingExampleAstar ------
add_executable(QuadtreePathfindingExampleAstar main.cpp)
target_include_directories(QuadtreePathfindingExampleAstar
                           PUBLIC "../../Source")
target_link_libraries(QuadtreePathfindingExampleAstar QDPF)
