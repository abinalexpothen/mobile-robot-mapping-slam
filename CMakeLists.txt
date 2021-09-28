project(mobile-robot-mapping-slam)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/measurement.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/poses.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_executable(occupanygridmapping scripts/occupancy_grid_mapping.cpp scripts/matplotlibcpp.h ${PYTHON_LIBRARIES}) 
target_link_libraries(occupanygridmapping python2.7)

add_executable(multisensorfusion scripts/multi_sensor_fusion.cpp)
target_link_libraries(multisensorfusion)