FROM ubuntu:16.04
# -- Update Ubuntu 16.04 --
RUN apt-get update && apt-get upgrade -y
# Install basic utilities
RUN apt-get install -y net-tools lsb-release \
    wget tree

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
# Source ROS installation (now handled by entrypoint)
# RUN source /opt/ros/kinetic/setup.bash

# DO NOT CHANGE ANYTHING ABOVE THIS (rebuild will take 3GB download!)
# -- Move the files to the ROS workspace(s) --
# Mount using `-v` manually
# - Workspace for RotorS -
COPY ./rotors_ws /ros_workspaces/rotors_ws/
# - Custom workspace for Docker container -
# COPY ./custom_ws /ros_workspaces/custom_ws/src/

# -- Set the entrypoint --
COPY ./docker/rotors_entrypoint.sh /rotors_entrypoint.sh
ENTRYPOINT [ "/rotors_entrypoint.sh" ]
