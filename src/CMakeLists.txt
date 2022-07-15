cmake_minimum_required(VERSION 2.8.3)
project(ntrip_ros)

find_package(catkin REQUIRED COMPONENTS
#  rtcm_msgs
  message_generation
  message_runtime
  #nmea_msgs
  rospy
  std_msgs
)

add_message_files(
  FILES
  GnssPVTSolnMsg.msg
  GnssTimeMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

