cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_simulator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(simulator_node src/simulator_node.cpp)


target_link_libraries(simulator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
