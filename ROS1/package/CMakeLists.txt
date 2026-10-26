cmake_minimum_required(VERSION 3.0.2)
project(drone_sorting)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  sensor_msgs
  message_filters
  tf2_msgs
)

# Enable the Python package defined in setup.py
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
    sensor_msgs
    message_filters
    tf2_msgs
)

# ROS executable nodes
catkin_install_python(
  PROGRAMS
    nodes/depth_mapping.py
    nodes/motion_controller.py
    nodes/perception.py
    nodes/mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install runtime configuration alongside launch files.
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
