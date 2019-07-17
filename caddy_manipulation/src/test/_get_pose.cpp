#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_caddy_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveGroupInterface move_group("arm_with_torso");
    ROS_INFO_STREAM(move_group.getCurrentPose());

    return 0;
}
