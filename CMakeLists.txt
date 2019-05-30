cmake_minimum_required(VERSION 2.8.3)
project(roboteq_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs serial geometry_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_node src/drive_node.cpp)

target_link_libraries(drive_node
   ${catkin_LIBRARIES}
)
