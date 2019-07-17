#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <fetch_cpp/FetchGripper.h>
#include <kit_detector/Caddy.h>
#include "OctomapBuilder.h"

using moveit::planning_interface::MoveGroupInterface;

class Placing {

public:
    Placing(MoveGroupInterface &move_group, OctomapBuilder& octomap_builder, geometry_msgs::PoseStamped& place_pose, double approach_z_diff);

    void approach(bool plan_only = false);

    virtual void release(bool plan_only = false);

    void lift_up();

    virtual void place();

private:

    MoveGroupInterface &move_group;

    FetchGripper gripper;

    OctomapBuilder& octomap_builder;

    geometry_msgs::PoseStamped approach_pose, place_pose;

    void setup_approach_pose(double approach_z_diff);
};