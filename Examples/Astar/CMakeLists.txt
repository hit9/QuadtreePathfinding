cmake_minimum_required(VERSION 3.10)

project(QuadtreePathfindingExampleAstar)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# ------ QDPF ---------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../Source
                 ${CMAKE_CURRENT_BINARY_DIR}/Source)

# ----- executable QuadtreePathfindingExampleAstar ------
add_executable(QuadtreePathfindingExampleAstar main.cpp)
target_include_directories(QuadtreePathfindingExampleAstar
                           PUBLIC "../../Source")
target_link_libraries(QuadtreePathfindingExampleAstar QDPF)
