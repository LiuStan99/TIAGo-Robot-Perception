# Cor Multidisciplinary Project - Tiago Simulation

<img src="https://img.shields.io/badge/ROS%20version-melodic-blue.svg"/>

Welcome to the Multidisciplinary Project - Tiago Simulation repository! This repository aims to be a simple one stop shop for simulating Tiago. All dependencies with their exact remote and version are listed in the `.rosinstall`. Using this it is possible to install locally.

**Important:** The only officially supported Ubuntu/ROS version is Bionic/Melodic for the Tiago simulation.

## Local

Create a catkin workspace and clone all required dependencies listed in `cor_mdp_tiago.rosinstall`. To automate the process [vcstool](http://wiki.ros.org/vcstool) can be used:

``` bash
mkdir -p <my_catkin_ws>/src # if no catkin_ws yet
cd <my_catkin_ws>/src
git clone https://gitlab.tudelft.nl/cor/ro47007/2022/team-<XX>/cor_mdp_tiago.git
vcs import --input cor_mdp_tiago/cor_mdp_tiago.rosinstall .
cd ..
```

> Note: replace the `<XX>` with your team number

Next, use rosdep to install other dependencies:
``` bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller" 
```
> Note: the skip-keys contain non-essential dependencies and are taken from the official [PAL install instructions](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS)

Finally build and source the workspace:
``` bash
catkin build && source devel/setup.bash
```

## Quickstart

### Ahold project

```
roslaunch cor_mdp_tiago_gazebo tiago_ahold.launch
```

![ahold world image](ahold_world.png)

### Festo project

```
roslaunch cor_mdp_tiago_gazebo tiago_festo.launch
```

![festo world image](festo_world.png)

## Marker Detection

In both the Festo and Ahold world, you may notice there are some items with markers on them. You can use these markers to easily detect the pose of the items. Please have a look at [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) to get a better understanding of how to use marker detection. The apriltag detection node is already included in both launch files.

**Important:** People who have followed the course KRR (Knowledge Reasoning and Representation) are likely familiar with Aruco markers, however the detection accuracy proved insufficient. Therefore, we have opted to use a different marker detection package by default for this course, see [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros). 

> Of course you are free to explore different marker detection packages, or even detection methods that don't require markers!


## Perception nodes

To activate perception nodes, run
```
roslaunch person_detector detector.launch
```

To visualize, in the second window, run

```
rosrun rqt_image_view rqt_image_view
```

In case you want to drive the robot, in the third window, run

```
rosrun key_teleop key_teleop.py
```

The detected people's locations are published via **vision_msgs/Detection3DArray**. 

http://docs.ros.org/en/api/vision_msgs/html/msg/Detection3DArray.html

The position of a person can be read from the bouding box center.

http://docs.ros.org/en/api/vision_msgs/html/msg/BoundingBox3D.html