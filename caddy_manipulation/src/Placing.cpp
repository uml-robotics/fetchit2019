#include "caddy_manipulation/Placing.h"

#include "caddy_manipulation/move_group_util.h"

using geometry_msgs::PoseStamped;

Placing::Placing(MoveGroupInterface &m, OctomapBuilder& ob, PoseStamped &place_pose, double approach_z_diff) : move_group(m), octomap_builder(ob), place_pose(place_pose) {
    setup_approach_pose(approach_z_diff);
}

void Placing::setup_approach_pose(double approach_z_diff) {
    approach_pose = place_pose;
    approach_pose.pose.position.z += approach_z_diff;

    tf::Transform t;
    tf::poseMsgToTF(approach_pose.pose, t);
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "approach_pose"));
}

void Placing::approach(bool plan_only) {
    // go tp approach phase
    move_group_util::set_pose_target_and_execute(move_group, approach_pose.pose, plan_only);
}

void Placing::release(bool plan_only) {
    octomap_builder.clear_map();
    move_group_util::set_pose_target_and_execute(move_group, place_pose.pose, plan_only);
    gripper.open();
}

void Placing::lift_up() {
    move_group_util::set_pose_target_and_execute(move_group, approach_pose.pose);
}

void Placing::place() {
    approach();
    release();
    lift_up();
    octomap_builder.restore_map();
}
