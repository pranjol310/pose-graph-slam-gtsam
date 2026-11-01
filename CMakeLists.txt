cmake_minimum_required(VERSION 3.10)

project(pose_graph_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTSAM REQUIRED)

add_executable(pose_graph_slam src/main.cpp)

target_link_libraries(pose_graph_slam PRIVATE gtsam)