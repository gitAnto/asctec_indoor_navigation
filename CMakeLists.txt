cmake_minimum_required(VERSION 2.8.3)
project(asctec_indoor_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  asctec_hl_comm
  std_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  dynamic_reconfigure
  roslib
)



 generate_dynamic_reconfigure_options(
   cfg/asctec_indoor_navigation.cfg
 )


catkin_package(
  INCLUDE_DIRS #include
#  LIBRARIES asctec_indoor_navigation
  CATKIN_DEPENDS actionlib asctec_hl_comm geometry_msgs std_msgs roscpp sensor_msgs tf dynamic_reconfigure roslib
  DEPENDS 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(waypoint_follower_node src/waypoint_follower.cpp src/waypoint_follower_node.cpp)


 target_link_libraries(waypoint_follower_node
   ${catkin_LIBRARIES}
 )


