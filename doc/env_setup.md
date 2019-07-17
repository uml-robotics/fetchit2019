# Environment Setup

Make sure Ubuntu 18.04 and ROS Melodic are installed. We are using this version because it is also installed on the robot and it works.

(Currently only ROS Indigo is supported as Debian packages, but there are git branches for Ubuntu 18.04/ROS Melodic and both are installed on the onboard computer of the robot.)

## Installing [fetch_ros](https://github.com/fetchrobotics/fetch_ros) & [fetch_gazebo](https://github.com/fetchrobotics/fetch_gazebo) from source

Install the dependencies for Fetch Robotics:

    sudo apt-get update
    sudo apt install ros-melodic-robot-controllers ros-melodic-costmap-2d ros-melodic-moveit ros-melodic-rgbd-launch

Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

In the carkin workspace `src` directory, run:

    git clone https://github.com/fetchrobotics/fetch_ros.git
    git clone https://github.com/fetchrobotics/fetch_gazebo.git

In your carkin workspace `root` directory, run

    catkin_make

## Test your installation in an empty world

    roslaunch fetch_gazebo simulation.launch

## Gazebo demo (optional, buggy)

Install dependencies:

    sudo apt install ros-melodic-map-server ros-melodic-simple-grasping ros-melodic-amcl ros-melodic-move-base ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers

### pick and place with navigation

This is documented here: https://docs.fetchrobotics.com/gazebo.html#running-the-mobile-manipulation-demo

    roslaunch fetch_gazebo playground.launch
    roslaunch fetch_gazebo_demo demo.launch

Zhao:

On my laptop without a dedicated GPU, which means slow, this will only make Fetch move near the first table. On the lab machines, it should be better.

I opened an issue in the fetch_gazebo repo: [#30](https://github.com/fetchrobotics/fetch_gazebo/issues/30).

There is another issue in that repo [#20](https://github.com/fetchrobotics/fetch_gazebo/issues/20). [The official reply](https://github.com/fetchrobotics/fetch_gazebo/issues/20#issuecomment-386675996) is in simulation there can be all sorts of problems.

### just pick and place

There is an undocumented launch without the navigation. The pick and place won't work on my laptop as well. I opened an issue here [#31](https://github.com/fetchrobotics/fetch_gazebo/issues/31).

    roslaunch fetch_gazebo pickplace_playground.launch
    roslaunch fetch_gazebo_demo pick_place_demo.launch

## Launch the competition environment

There is a full version in an unnecessary factory environment. The origin is the center of the factory room.

```bash
roslaunch fetchit_challenge main.launch
```

but this `main_simple.launch` is closer to the real arena and our test course at Prof. Reza's lab. The origin is the center of the competition environment; it's probably better for the ros navigation stack.

```bash
roslaunch fetchit_challenge main_simple.launch
```

<hr>

References:
- http://docs.fetchrobotics.com/gazebo.html
- https://bitbucket.org/theconstructcore/fetchit_notebook
