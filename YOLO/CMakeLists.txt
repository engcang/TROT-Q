cmake_minimum_required(VERSION 2.8.3)
project(yolo_ros_simple)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
    bbox.msg
    bboxes.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()