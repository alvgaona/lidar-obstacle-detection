cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

project(cluster)

find_package(PCL 1.2 REQUIRED)

add_executable (cluster cluster.cpp ../../render.cpp)
target_include_directories(cluster PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(cluster PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_definitions(cluster PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cluster PRIVATE ${PCL_LIBRARIES})
