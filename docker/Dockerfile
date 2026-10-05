# Base image running ROS 1 Noetic on Ubuntu 20.04
FROM ros:noetic-ros-base

# Install git, build tools, and the python-is-python3 alias
RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-pip \
    python-is-python3 \
    && rm -rf /var/lib/apt-get/lists/*

# Set ROS_IP so Unity ROS-TCP-Endpoint listens on 0.0.0.0 inside container
ENV ROS_IP=0.0.0.0

# Create ROS catkin workspace source folder
WORKDIR /catkin_ws/src

# Download required third-party Unity ROS-TCP-Endpoint package into container
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Copy your local ROS package from host into the container workspace
COPY ROS1/src/drone_sorting ./drone_sorting

# Build Catkin workspace
WORKDIR /catkin_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Automatically source workspace environment on login
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Expose Unity ROS Settings default TCP port
EXPOSE 10000

# Run ROS launch file on container start
CMD ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && roslaunch drone_sorting drone_demo.launch"]