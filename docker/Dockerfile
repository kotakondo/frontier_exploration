FROM osrf/ros:noetic-desktop-full

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for Active CSLAM"

RUN rm -rf /var/lib/apt/lists/*
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -q -y --no-install-recommends git tmux vim apt-utils wget && \
    apt-get autoremove -y && \
    apt-get clean

# ROS dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-noetic-joy -y \
                    ros-noetic-teleop-twist-joy  \
                    ros-noetic-teleop-twist-keyboard \
                    ros-noetic-laser-proc \
                    ros-noetic-rgbd-launch -y \
                    ros-noetic-rosserial-arduino -y \
                    ros-noetic-rosserial-python \
                    ros-noetic-rosserial-client \
                    ros-noetic-rosserial-msgs \
                    ros-noetic-amcl \
                    ros-noetic-map-server -y \
                    ros-noetic-move-base -y \
                    ros-noetic-urdf -y \
                    ros-noetic-xacro \
                    ros-noetic-compressed-image-transport \
                    ros-noetic-rqt -y \
                    ros-noetic-rqt-common-plugins -y \
                    ros-noetic-rqt-gui-cpp \
                    ros-noetic-rqt-gui-cpp-dbgsym \
                    ros-noetic-rqt-human-radar -y \
                    ros-noetic-rqt-human-radar-dbgsym \
                    ros-noetic-rqt-image-view \
                    ros-noetic-rqt-image-view-dbgsym \
                    ros-noetic-rqt-multiplot -y \
                    ros-noetic-rqt-multiplot-dbgsym \
                    ros-noetic-rqt-robot-plugins -y \
                    ros-noetic-rqt-rosbag-fancy -y \
                    ros-noetic-rqt-rosbag-fancy-dbgsym \
                    ros-noetic-rqt-rosmon -y \
                    ros-noetic-rqt-rosmon-dbgsym \
                    ros-noetic-rqt-rviz -y \
                    ros-noetic-rqt-rviz-dbgsym -y \
                    ros-noetic-rviz -y \
                    ros-noetic-gmapping -y \
                    ros-noetic-navigation -y \
                    ros-noetic-interactive-markers -y \
                    ros-noetic-gazebo-ros -y \
                    ros-noetic-tf2-sensor-msgs -y \
                    ros-noetic-mbf-costmap-core -y \
                    ros-noetic-mbf-msgs -y \
                    -y '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev

# Install turtlebot3
RUN apt install ros-noetic-dynamixel-sdk
RUN apt install ros-noetic-turtlebot3-msgs

# Set up Robot model and realsense
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN apt-get install ros-noetic-librealsense2* -y
RUN apt-get install ros-noetic-ddynamic-reconfigure

# INSTALL python-catkin-tools (to be able to use catkin build)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - |  apt-key add -
RUN apt-get install python3-catkin-tools -y

# Misc
RUN apt install ros-noetic-camera-info-manager* -y \
                tmuxp -y \
                libxcb-* -y

# Install repos
RUN mkdir -p /home/code/catkin_ws/src
WORKDIR /home/code/catkin_ws/src
COPY turtlebot3_simulations /home/code/catkin_ws/src/turtlebot3_simulations
COPY frontier_exploration /home/code/catkin_ws/src/frontier_exploration
COPY turtlebot3 /home/code/catkin_ws/src/turtlebot3
COPY navigation /home/code/catkin_ws/src/navigation
COPY realsense-ros /home/code/catkin_ws/src/realsense-ros
COPY realsense_gazebo_plugin /home/code/catkin_ws/src/realsense_gazebo_plugin
COPY costmap_converter /home/code/catkin_ws/src/costmap_converter

# Ros dependencies
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/code/catkin_ws/src --ignore-src -r -y
RUN catkin config -DCMAKE_BUILD_TYPE=Release

# Build
SHELL ["/bin/bash", "-c"]
WORKDIR /home/code/catkin_ws/
RUN . ~/.bashrc && . /opt/ros/noetic/setup.sh && catkin build
RUN echo "source /home/code/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set up entrypoint
WORKDIR /home/code/catkin_ws/src/frontier_exploration/tmux
ENTRYPOINT [ "tmuxp", "load", "sim.yaml" ]