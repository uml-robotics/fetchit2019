roscore

rosparam set robot_description "$(cat $(rospack find fetch_description)/robots/fetch.urdf)"
rosparam set use_sim_time true
rosbag play ~/Downloads/schunk_machine_close_large_gears_2019-04-16-17-16-59.bag --clock

roscd gear_detector && rviz -d rviz.rviz