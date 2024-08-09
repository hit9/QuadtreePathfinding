cmake_minimum_required(VERSION 3.10)

project(quadtree_astar)

add_library(quadtree_astar SHARED quadtree_astar.cpp)
set_target_properties(quadtree_astar PROPERTIES PUBLIC_HEADER
    "quadtree_astar.hpp")
include_directories("3rd-party")

install(
    TARGETS quadtree_astar
    LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
