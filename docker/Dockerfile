FROM ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for Active CSLAM"

RUN rm -rf /var/lib/apt/lists/*
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -q -y --no-install-recommends git tmux vim apt-utils && \
    apt-get install -q -y --no-install-recommends wget && \
    apt-get autoremove -y && \
    apt-get clean

# ROS dependencies
RUN apt-get install ros-noetic-joy -y
RUN apt-get install ros-noetic-teleop-twist-joy 
RUN apt-get install ros-noetic-teleop-twist-keyboard  
RUN apt-get install ros-noetic-laser-proc 
RUN apt-get install ros-noetic-rgbd-launch -y
RUN apt-get install ros-noetic-rosserial-arduino -y
RUN apt-get install ros-noetic-rosserial-python  
RUN apt-get install ros-noetic-rosserial-client 
RUN apt-get install ros-noetic-rosserial-msgs  
RUN apt-get install ros-noetic-amcl  
RUN apt-get install ros-noetic-map-server -y
RUN apt-get install ros-noetic-move-base -y
RUN apt-get install ros-noetic-urdf -y
RUN apt-get install ros-noetic-xacro 
RUN apt-get install ros-noetic-compressed-image-transport  
RUN apt-get install ros-noetic-rqt -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-noetic-rqt-common-plugins -y
RUN apt-get install ros-noetic-rqt-gui-cpp  
RUN apt-get install ros-noetic-rqt-gui-cpp-dbgsym 
RUN apt-get install ros-noetic-rqt-human-radar -y
RUN apt-get install ros-noetic-rqt-human-radar-dbgsym 
RUN apt-get install ros-noetic-rqt-image-view
RUN apt-get install ros-noetic-rqt-image-view-dbgsym 
RUN apt-get install ros-noetic-rqt-multiplot -y
RUN apt-get install ros-noetic-rqt-multiplot-dbgsym 
RUN apt-get install ros-noetic-rqt-robot-plugins -y
RUN apt-get install ros-noetic-rqt-rosbag-fancy -y
RUN apt-get install ros-noetic-rqt-rosbag-fancy-dbgsym 
RUN apt-get install ros-noetic-rqt-rosmon -y
RUN apt-get install ros-noetic-rqt-rosmon-dbgsym 
RUN apt-get install ros-noetic-rqt-rviz -y
RUN apt-get install ros-noetic-rqt-rviz-dbgsym  -y
RUN apt-get install ros-noetic-rviz -y
RUN apt-get install ros-noetic-gmapping -y  
RUN apt-get install ros-noetic-navigation -y 
RUN apt-get install ros-noetic-interactive-markers -y

# Install turtlebot3
RUN apt install ros-noetic-dynamixel-sdk
RUN apt install ros-noetic-turtlebot3-msgs

# Set up Robot model
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN apt-get install ros-noetic-librealsense2* -y
RUN apt-get install ros-noetic-ddynamic-reconfigure

#INSTALL python-catkin-tools (to be able to use catkin build)
##########################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - |  apt-key add -
RUN apt-get install python3-catkin-tools -y

RUN apt-get install ros-noetic-gazebo-ros -y
RUN apt install ros-noetic-camera-info-manager* -y
RUN apt-get install ros-noetic-tf2-sensor-msgs -y
RUN apt-get install ros-noetic-mbf-costmap-core -y
RUN apt-get install ros-noetic-mbf-msgs -y
RUN apt install tmuxp -y
RUN apt install libxcb-* -y
RUN apt-get install -y '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev

RUN apt install python3-pip -y
RUN apt install python3-tk -y
RUN apt-get install ros-noetic-desktop-full -y
RUN apt-get install python3-rosdep -y
RUN apt-get install python3-rosinstall -y
RUN apt-get install python3-rosinstall-generator -y
RUN apt-get install python3-wstool -y
RUN apt-get install build-essential -y
RUN apt-get install ros-noetic-rosbash -y
RUN apt-get install ros-noetic-depthimage-to-laserscan -y
RUN apt-get install ros-noetic-rosserial-server -y
RUN apt-get install ros-noetic-robot-state-publisher -y
RUN sudo apt-get install ros-noetic-pointcloud-to-laserscan

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

# Ros stuff
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/code/catkin_ws/src --ignore-src -r -y
RUN catkin config -DCMAKE_BUILD_TYPE=Release

# Build
WORKDIR /home/code/catkin_ws
SHELL ["/bin/bash", "-c"]
RUN . ~/.bashrc && . /opt/ros/noetic/setup.sh && catkin build
RUN echo "source /home/code/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set up entrypoint
ENTRYPOINT [ "tmuxp", "load", "src/frontier_exploration/tmux/sim.yaml" ]