```bash
roslaunch fetchit_challenge main_arena_montreal2019_highlights.launch

# move the robot in front of schunk table
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: -0.4, y: -0.3, z: 0 }, orientation: {x: 0, y: 0, z: 3.1415, w: 0 } }}'

# move the robot in front of schunk table, right edge
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: -0.4, y: 0.15, z: 0 }, orientation: {x: 0, y: 0, z: 3.1415, w: 0 } }}'

#rviz
roscd chuck_detector && rviz -d chuck.rviz
```



```c++
lift torso
headClient.relatively_look_at(1, 0.5, -0.4);
```