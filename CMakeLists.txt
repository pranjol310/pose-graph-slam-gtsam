cmake_minimum_required(VERSION 3.10)

project(pose_graph_slam)

find_package(GTSAM REQUIRED)

add_executable(pose_graph_slam src/main.cpp)
target_link_libraries(pose_graph_slam gtsam)

add_executable(isam2 src/isam2.cpp)
target_link_libraries(isam2 gtsam)