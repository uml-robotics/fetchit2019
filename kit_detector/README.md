# kit_detector

A package to detect the caddy given the scene cloud containing the table area.

## How to use this package

You can use it as other ROS packages like `pcl_ros` but just need to build it by `catkin_make --pkg kit_detector` or build the whole workspace `catkin_make`.

Here is how you can use it like any other ROS packages.

```cmake
find_package(catkin REQUIRED COMPONENTS
  kit_detector
)
```

Then you can `#include <kit_detector/xxx.h>`.

## How to run in development

Make sure you followed [env_setup.md](../doc/env_setup.md) and launched `roslaunch fetchit_challenge main.launch`.

### Move robot to  kit station

`x=3.5, y=-3.4` to see the whole table

```bash

rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: 3.5, y: -3.4, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }}'
```

### Open rviz

```bash
roscd kit_detector && rviz -d kit_detector.rviz
```

### Open clion

```bash
roscd kit_detector && clion .
```

## Commands for testing clustering

roslaunch kit_detector three_caddies.launch

rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: 4, y: -3.4, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }}'

roscd kit_detector && rviz -d kit_detector.rviz
