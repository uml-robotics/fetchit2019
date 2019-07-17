# caddy_manipulation

## run on real robot

### remote

tmux

### both ok

roslaunch caddy_manipulation move_group.launch allow_active_sensing:=true moveit_octomap_sensor_params_file:=$(rospack find caddy_manipulation)/config/octomap.yaml

### local

roscd caddy_manipulation && rviz -d moveit.rviz

## Commands for testing (grasp_caddy_test & place_parts_test)

```bash
# one caddy
roslaunch fetchit_challenge main.launch

# multiple caddies (3)
roslaunch kit_detector three_caddies.launch
```


```bash
# make sure torso is lifted & the robot is looking at the caddy
#  first change in KitDetector.cpp
#  headClient.relatively_look_at(0.73-0.5, 0, -0.14);

rosrun kit_detector kit_detector_test
```

rosrun tf static_transform_publisher 0 0 0 0 0 0 world odom 10

```bash
# launch move_group

# now
roslaunch caddy_manipulation move_group.launch allow_active_sensing:=true moveit_octomap_sensor_params_file:=$(rospack find caddy_manipulation)/config/octomap.yaml

# if PR merged (https://github.com/fetchrobotics/fetch_ros/pull/116)

roslaunch fetch_moveit_config move_group.launch allow_active_sensing:=true moveit_octomap_sensor_params_file:=$(rospack find caddy_manipulation)/config/octomap.yaml
```

roscd caddy_manipulation && rviz -d moveit.rviz

```bash
# move robot

# in front of desk
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: 3.75, y: -3.4, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }}'

# in front of caddy
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: 4, y: -3.6, z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }}'
```

```
# reset
rosrun fetch_gazebo prepare_simulated_robot_pick_place.py
rosrun caddy_manipulation look_at
rosrun caddy_manipulation raise_torso
# move to in front of caddy
```