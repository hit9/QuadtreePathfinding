cmake_minimum_required(VERSION 3.10)

project(quadtree_pathfinding)

add_library(quadtree_pathfinding SHARED quadtree_pathfinding.cpp)
set_target_properties(quadtree_pathfinding
    PROPERTIES PUBLIC_HEADER "quadtree_pathfinding.hpp")
include_directories("3rd-party")

install(
    TARGETS quadtree_pathfinding
    LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
