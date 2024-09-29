cmake_minimum_required(VERSION 3.10)

project(QDPF)

include(FetchContent)

message(STATUS "Fetching https://github.com/hit9/ClearanceField.git...")
FetchContent_Declare(
  ClearanceField
  GIT_REPOSITORY https://github.com/hit9/ClearanceField.git
  GIT_TAG v0.4.1
  SOURCE_SUBDIR Source SOURCE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/_deps/ClearanceField)

message(STATUS "Fetching https://github.com/hit9/Quadtree-hpp.git...")
FetchContent_Declare(
  Quadtree
  GIT_REPOSITORY https://github.com/hit9/Quadtree-hpp.git
  GIT_TAG v0.4.1
  SOURCE_SUBDIR Source SOURCE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/_deps/Quadtree-hpp)

FetchContent_MakeAvailable(ClearanceField)
FetchContent_MakeAvailable(Quadtree)

file(GLOB_RECURSE QDPF_SOURCES Internal/*.cpp Naive/*.cpp QDPF.cpp)
add_library(QDPF SHARED ${QDPF_SOURCES})
target_include_directories(QDPF PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/_deps)
target_link_libraries(QDPF ClearanceField)
set_target_properties(QDPF PROPERTIES PUBLIC_HEADER "QDPF.h")

install(
  TARGETS QDPF
  LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
