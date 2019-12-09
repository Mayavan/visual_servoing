cmake_minimum_required(VERSION 2.8.3)
project(visual_servoing)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ARMarker.msg
)

add_service_files(
  FILES
  getTarget.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs visualization_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(path_planning_node src/path_planning_node.cpp src/ur5_control.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(path_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(path_planning_node ${catkin_LIBRARIES})
