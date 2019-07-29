cmake_minimum_required(VERSION 2.8.3)
project(hospitality)

find_package(catkin REQUIRED roscpp rospy std_msgs geometry_msgs sensor_msgs message_generation)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
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

add_executable(turtlebot3_drive_node src/turtlebot3_drive.cpp)
target_link_libraries(turtlebot3_drive_node ${catkin_LIBRARIES})
