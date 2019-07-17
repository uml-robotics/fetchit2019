This is the position that I use for opening and closing the schunk machine door:

rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: -0.4, y: -0.2, z: 0 }, orientation: {x: 0, y: 0, z: 3.1415, w: 0 } }}
