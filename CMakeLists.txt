cmake_minimum_required(VERSION 3.14)
project(spatial_transform_tree VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_library(spatial_tree STATIC src/bsps_protocol.c)
target_include_directories(spatial_tree PUBLIC include)

add_executable(spatial_example src/main.cpp)
target_link_libraries(spatial_example PRIVATE spatial_tree)
