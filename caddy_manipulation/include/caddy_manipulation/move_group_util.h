#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace move_group_util {

    bool ask_();

    void follow_traj(
            moveit::planning_interface::MoveGroupInterface &move_group,
            const std::vector<geometry_msgs::Pose> &waypoints,
            bool plan_only = false);

    bool set_pose_target_and_execute(
            moveit::planning_interface::MoveGroupInterface &move_group,
            const geometry_msgs::Pose &target,
            const int MAX_RETRY = 3,
            bool plan_only = false);


    bool set_joints_target_and_execute(
            moveit::planning_interface::MoveGroupInterface &move_group,
            std::vector<std::string> joints,
            std::vector<double> values,
            const int MAX_RETRY = 3,
            bool plan_only = false);
}