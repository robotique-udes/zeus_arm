cmake_minimum_required(VERSION 3.0.2)
project(zeus_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

#add_message_files(
#   FILES
   
# )


 
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

