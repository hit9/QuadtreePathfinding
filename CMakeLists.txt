cmake_minimum_required(VERSION 3.10)

project(quadtree_pathfinding)

include_directories("3rd-party")
file(GLOB QDPF_SOURCES qdpf.cpp internal/*.cpp)
add_library(quadtree_pathfinding SHARED ${QDPF_SOURCES})
set_target_properties(quadtree_pathfinding PROPERTIES PUBLIC_HEADER "qdpf.hpp")

install(
    TARGETS quadtree_pathfinding
    LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
