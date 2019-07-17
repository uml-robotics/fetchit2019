tmux:
	tmux new "tmux source-file config/competition.tmux.conf"

record_downsample:
	rosbag record /head_camera/depth_downsample/points /tf /tf_static /joint_states --duration 10

record_registered:
	rosbag record /head_camera/depth_registered/points /tf /tf_static /joint_states --limit=10