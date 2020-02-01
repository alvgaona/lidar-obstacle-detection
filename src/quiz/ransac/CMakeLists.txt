cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

project(ransac)

find_package(PCL 1.2 REQUIRED)

file(GLOB SOURCES src/*.cpp)

add_executable(ransac ransac2d.cpp ../../render.cpp)
target_include_directories(ransac PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(ransac PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_definitions(ransac PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(ransac PRIVATE ${PCL_LIBRARIES})
