cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node src/path_planner_node.cpp)

target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
)

