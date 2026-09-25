cmake_minimum_required(VERSION 3.16)
project(hybrid_astar CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(demo
    src/main.cpp
    src/grid_map.cpp
    src/astar.cpp
)

target_include_directories(demo PRIVATE include)
target_compile_options(demo PRIVATE -Wall -Wextra)
