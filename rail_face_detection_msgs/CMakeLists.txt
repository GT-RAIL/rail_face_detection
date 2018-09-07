cmake_minimum_required(VERSION 2.8.3)
project(rail_face_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    Face.msg
    Detections.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs std_msgs
)
