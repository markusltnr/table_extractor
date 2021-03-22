cmake_minimum_required(VERSION 2.8.3)
project(table_extractor)

find_package(catkin REQUIRED
message_generation
message_runtime
std_msgs
geometry_msgs
rospy
roscpp
sensor_msgs

)

add_message_files(FILES
   Plane.msg
   Table.msg
   TableViewpoints.msg
)

generate_messages(
   DEPENDENCIES
   
   std_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(CATKIN_DEPENDS 
    roscpp 
    rospy
    std_msgs
    message_generation
)

