cmake_minimum_required(VERSION 3.0.2)
project(controll)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  message_generation
)

add_service_files(
  FILES
  Controll.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/flight_node_test.py
  scripts/motion_server.py
  scripts/square_timer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
