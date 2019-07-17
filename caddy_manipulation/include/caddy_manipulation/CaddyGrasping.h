#pragma once

#include "kit_detector/Caddy.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fetch_cpp/FetchGripper.h>

using moveit::planning_interface::MoveGroupInterface;

class CaddyGrasping {

public:
    constexpr static const double LIFT_OFFSET = Caddy::height + 0.1;

    CaddyGrasping(MoveGroupInterface &move_group, geometry_msgs::PoseStamped& handle_center, geometry_msgs::PoseStamped& grasp_pose);

    bool approach(bool plan_only = false, double gripper_open_position = FetchGripper::MAX, double approach_z_diff = 0.05);

    bool grasp(bool plan_only);

    bool lift_up(bool plan_only = false, double lift_offset = LIFT_OFFSET);

private:

    MoveGroupInterface &move_group;

    FetchGripper gripper;

    geometry_msgs::PoseStamped grasp_pose;
};
