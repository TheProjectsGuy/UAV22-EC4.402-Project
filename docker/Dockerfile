FROM ubuntu:16.04
# -- Update Ubuntu 16.04 --
RUN apt-get update && apt-get upgrade -y
# Install basic utilities
RUN apt-get install -y net-tools lsb-release \
    wget

# -- Install ROS in the container --
# Add ROS in list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    > /etc/apt/sources.list.d/ros-latest.list
# Add the key for ROS
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get update
# Install the desktop version (full)
RUN apt-get install -y ros-kinetic-desktop-full ros-kinetic-joy \
    ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool \
    python-catkin-tools protobuf-compiler libgoogle-glog-dev \
    ros-kinetic-control-toolbox ros-kinetic-mavros
# Initialize ROS dependencies
RUN rosdep init
RUN rosdep update
# Source ROS installation
# RUN source /opt/ros/kinetic/setup.bash
