cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

project(lidar_obstacle_detection)

find_package(PCL 1.2 REQUIRED)

file(GLOB SOURCES src/*.cpp)

add_executable(environment ${SOURCES})
add_definitions(${PCL_DEFINITIONS})
target_include_directories(environment PRIVATE
    include
    ${PCL_INCLUDE_DIRS})
target_link_directories(environment PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(environment PRIVATE ${PCL_LIBRARIES})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
