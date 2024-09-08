cmake_minimum_required(VERSION 3.10)

project(quadtree_pathfinding)

include_directories("3rd-party")

file(GLOB TRUE_CLEARANCE_FIELD_SOURCES 3rd-party/true-clearance-field/*.cpp)
add_library(true_clearance_field ${TRUE_CLEARANCE_FIELD_SOURCES})

file(GLOB QDPF_SOURCES qdpf.cpp internal/*.cpp)
add_library(quadtree_pathfinding SHARED ${QDPF_SOURCES})
target_link_libraries(quadtree_pathfinding true_clearance_field)
set_target_properties(quadtree_pathfinding PROPERTIES PUBLIC_HEADER "qdpf.hpp")

install(
  TARGETS quadtree_pathfinding
  LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
