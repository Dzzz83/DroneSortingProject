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
    scripts/mission_manager_node.py
    scripts/motion_controller_node.py
    scripts/camera_stream_check_node.py
    scripts/package_perception_node.py
    scripts/camera_pickup_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install ROS launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
