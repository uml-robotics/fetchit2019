#pragma once

#include "kit_detector/Caddy.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fetch_cpp/FetchGripper.h>

using moveit::planning_interface::MoveGroupInterface;

class CaddyGrasping {

public:
    constexpr static const double LIFT_OFFSET = Caddy::height + 0.05;

    CaddyGrasping(MoveGroupInterface &move_group, geometry_msgs::PoseStamped& handle_center, geometry_msgs::PoseStamped& grasp_pose);

    void approach(bool plan_only = false);

    void grasp(bool plan_only);

    void lift_up(bool plan_only = false);

private:

    MoveGroupInterface &move_group;

    FetchGripper gripper;

    geometry_msgs::PoseStamped grasp_pose;
};
