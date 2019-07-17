# navigation

Official doc: http://docs.fetchrobotics.com/navigation.html

## rviz

roscd navigation/config/default && rviz -d *rviz

## Build & save map

 

roslaunch fetch_navigation build_map.launch

roscd navigation/maps/reza-lab/

rosrun map_server map_saver -f # filename

## navigation.launch
roslaunch navigation navigation.launch mapfile:=$(rospack find navigation)/maps/reza-lab/5-6.yaml