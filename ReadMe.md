# Adaptive Spatio-Temporal Voxels Based Trajectory Planning for Autonomous Driving in Highway Traffic Flow

This repo is the code for the paper "Adaptive Spatio-Temporal Voxels Based Trajectory Planning for Autonomous Driving in Highway Traffic Flow".

https://user-images.githubusercontent.com/20518317/235564429-abe2bb56-9fa1-48ae-b82a-2a1c8a98018b.mp4

# Requirements

For running code:

- Ubuntu20.04

- python3.9

- ros-noetic (http://wiki.ros.org/ROS/Installation)

- rospkg (pip install rospkg)

- gym (pip install gym)

- pygame (pip install pygame)

- OOQP (https://pages.cs.wisc.edu/~swright/ooqp/)

- glog (https://github.com/google/glog)

- pcl_ros (sudo apt install ros-noetic-pcl-ros)

# Install

1. create a root directory

```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```

2. clone the code

```
git clone https://github.com/flztiii/traffic_flow_trajectory_planning.git
```

3. install

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# How to use

1. start roscore

```
roscore
```

2. launch the simulator

```
cd catkin_ws
source devel/setup.bash
roslaunch simulator start_gym.launch
```

3. start the planner

```
cd catkin_ws
source devel/setup.bash
roslaunch local_planning test.launch
```

press "Enter" key in the terminal to start the program.

# Acknowledgement

https://github.com/HKUST-Aerial-Robotics/EPSILON

https://github.com/Farama-Foundation/HighwayEnv