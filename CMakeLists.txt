cmake_minimum_required(VERSION 3.16)
project(hybrid_astar CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CORE_SOURCES
    src/grid_map.cpp
    src/astar.cpp
    src/hybrid_astar.cpp
    src/image.cpp
)

add_executable(demo src/main.cpp ${CORE_SOURCES})
target_include_directories(demo PRIVATE include)
target_compile_options(demo PRIVATE -Wall -Wextra)

add_executable(run_tests tests/test_main.cpp ${CORE_SOURCES})
target_include_directories(run_tests PRIVATE include)
target_compile_options(run_tests PRIVATE -Wall -Wextra)

enable_testing()
# 地图文件用相对路径加载，测试的工作目录设为项目根目录
add_test(NAME core COMMAND run_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
