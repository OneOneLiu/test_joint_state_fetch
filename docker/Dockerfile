FROM osrf/ros:humble-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive

# System layer installation
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y xauth x11-apps # This is for GUI testing

# Install realsense2_camera
RUN apt install -y ros-$ROS_DISTRO-librealsense2* && \
    apt install -y ros-$ROS_DISTRO-realsense2-*

# For correct configure QT library on the remote server
RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
# Build moveit2 from source 
# 1. https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html
# 2. https://www.youtube.com/watch?v=c6Bxbq8UdaI
# Note: Pre compile here because it takes a long time (~18 minutes) to compile
RUN rosdep update && \
    apt update && \
    apt -y dist-upgrade && \
    apt install python3-vcstool && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone --branch humble https://github.com/moveit/moveit2_tutorials && \
    vcs import < moveit2_tutorials/moveit2_tutorials.repos && \
    rosdep update && \
    apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && \
    /bin/bash -c ". /opt/ros/humble/setup.bash; cd /catkin_ws; colcon build --mixin release --executor sequential"

# Install UR5 robot
RUN cd /catkin_ws/src && \
    apt install -y ros-${ROS_DISTRO}-ros2-control && \
    apt install -y ros-${ROS_DISTRO}-ament-cmake && \
    apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    apt-get install -y ros-${ROS_DISTRO}-ur && \
    apt-get install -y ros-${ROS_DISTRO}-ur-bringup

RUN cd /catkin_ws/ && \
    rosdep update && \
    /bin/bash -c ". /opt/ros/humble/setup.bash;. /catkin_ws/install/setup.bash" && \
    apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    # /bin/bash -c ". /opt/ros/humble/setup.bash; colcon build --packages-ignore moveit*"

# Automatically source
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/install/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"