cmake_minimum_required(VERSION 2.8.3)
project(hospitality_server)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED 
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  hospitality_msgs
)

find_package(
  Eigen3 REQUIRED
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   PointFloor.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   FindRoute.srv
#   UpdatePosition.srv
#   UpdateURI.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hospitality
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(robot_manager_node src/robot_manager_node.cpp)
target_link_libraries(robot_manager_node ${catkin_LIBRARIES})

add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node ${catkin_LIBRARIES})