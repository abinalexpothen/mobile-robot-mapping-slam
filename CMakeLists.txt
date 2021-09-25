project(mobile-robot-mapping-slam)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/measurement.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/poses.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
add_executable(mobilerobotmapping scripts/occupancy_grid_mapping.cpp)
target_link_libraries(mobilerobotmapping)