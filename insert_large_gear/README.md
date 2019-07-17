roslaunch caddy_manipulation move_group.launch allow_active_sensing:=true moveit_octomap_sensor_params_file:=$(rospack find caddy_manipulation)/config/octomap.yaml

roscd caddy_manipulation && rviz -d moveit.rviz
