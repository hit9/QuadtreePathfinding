cmake_minimum_required(VERSION 3.10)

project(QDPF)

include_directories("3rdParty")

file(GLOB CLEARANCE_FIELD_SOURCES 3rdParty/ClearanceField/*.cpp)
add_library(ClearanceField ${CLEARANCE_FIELD_SOURCES})

file(GLOB_RECURSE QDPF_SOURCES *.cpp)
add_library(QDPF SHARED ${QDPF_SOURCES})
target_link_libraries(QDPF ClearanceField)
set_target_properties(QDPF PROPERTIES PUBLIC_HEADER "QDPF.h")

install(
  TARGETS QDPF
  LIBRARY DESTINATION ${CMAK_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
