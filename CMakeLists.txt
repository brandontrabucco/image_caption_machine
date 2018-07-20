cmake_minimum_required(VERSION 2.8.3)
project(image_caption_machine)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  rt_msgs
  world_msgs
  message_generation
  cv_bridge
  image_transport
  image_geometry
  tf
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  ImageBytes.msg
  WorldPlace.msg
)

add_service_files(
  FILES
  Caption.srv
  Machine.srv
  WorldAppend.srv
  WorldLookup.srv
  WorldString.srv
  WorldLength.srv
  WorldClosest.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  rt_msgs
  world_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_caption_machine
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs world_msgs std_msgs rt_msgs rospy cv_bridge image_transport
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
	PROGRAMS
	  nodes/world_server.py
	  nodes/caption_server.py
	  nodes/alexa_server.py
	  tests/machine_tests.py
	DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)
