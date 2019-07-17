# Working with the real robot

## To start

```
ssh fetch@fetch1068
```

## Update our repo (if update is needed)

I added the public key of the robot as deploy key (read only access) to our repo, so we can pull from the robot.

```
cd ~/ros_ws/src/fetchit/

git pull

catkin_make
```

## Local

`export ROS_MASTER_URI=http://fetch1068:11311/`

## Software runstop

Enable: `rostopic pub /enable_software_runstop std_msgs/Bool True --once`

Disable: `rostopic pub /enable_software_runstop std_msgs/Bool False --once`

## Teleop

### PS3 Controller

http://docs.fetchrobotics.com/teleop.html

### Keyboard

Move around: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Open/Close gripper: `rosrun fetch_teleop gripper_keyboard.py`

Tuck arm: `rosrun fetch_teleop tuck_arm.py`

## If connected through Ethernet

1. Use `arp` to find/verify IP of the robot (10.42.0.55) & add entry to `/etc/hosts`
2. Set ROS_IP: `export ROS_IP=10.42.0.1` (verify using `ifconfig enp4s1`)