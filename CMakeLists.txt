cmake_minimum_required(VERSION 2.8.3)
project(baxter_myo)

find_package(catkin
  REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  tf
  baxter_core_msgs
  baxter_interface
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  MyoData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  tf
  baxter_core_msgs
  baxter_interface
  message_runtime
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
