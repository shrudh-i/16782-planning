cmake_minimum_required(VERSION 3.16.3)
project(planning_hw3)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(planner src/planner.cpp)

target_compile_definitions(planner PRIVATE ENVS_DIR="${CMAKE_SOURCE_DIR}/envs")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)