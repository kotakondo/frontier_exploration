# frontier_exploration

Thread safe frontier exploration package based on fast and efficient Wavefront Frontier Detection.

## Resolving dependencies

Use `rosdep` to resolve ROS dependencies

## References

```
@misc{topiwala2018frontier,
      title={Frontier Based Exploration for Autonomous Robot}, 
      author={Anirudh Topiwala and Pranav Inani and Abhishek Kathpal},
      year={2018},
      eprint={1806.03581},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Set up

- clone this repository into your catkin workspace:
```git clone https://github.com/kotakondo/frontier_exploration.git```
 
- install dependence ros packages (ref: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
```sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

- install turtlebot3 packages (ref: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
```sudo apt install ros-noetic-dynamixel-sdk
      sudo apt install ros-noetic-turtlebot3-msgs
      sudo apt install ros-noetic-turtlebot
```

- install simulation package in `src` folder (ref: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
```git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git```

- set up tortlebot3 model (ref: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
```echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc``` or 
```echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc``` or 
```echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc```

```source ~/.bashrc```

## Demo using Turtlebot3

- **Shell #1** : Gazebo

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

- **Shell #2** : SLAM + RViz

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

- **Shell #3** : move_base

```bash
roslaunch turtlebot3_navigation move_base.launch 
```

- **Shell #4** : Frontier exploration

```bash
roslaunch frontier_exploration explore_costmap.launch
```