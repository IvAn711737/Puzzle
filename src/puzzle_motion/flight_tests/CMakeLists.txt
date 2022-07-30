cmake_minimum_required(VERSION 3.0.2)
project(flight_tests)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/flight_through_points_test.py
  scripts/position_hold_test.py
  scripts/rotate_angle_test.py
  scripts/spin_laps_test.py
  scripts/takeoff_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
