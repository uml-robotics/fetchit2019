#include "caddy_manipulation/CaddyGrasping.h"

#include <tf/transform_broadcaster.h>
#include "caddy_manipulation/move_group_util.h"

CaddyGrasping::CaddyGrasping(
        MoveGroupInterface &move_group,
        geometry_msgs::PoseStamped &handle_center,
        geometry_msgs::PoseStamped &grasp_pose)
        : move_group(move_group), grasp_pose(grasp_pose)
{
}


bool CaddyGrasping::approach(bool plan_only, double gripper_open_position, double approach_z_diff) {
    if (! plan_only)
        gripper.open(gripper_open_position);

    auto approach_pose = grasp_pose;
    approach_pose.pose.position.z += approach_z_diff;

    tf::Transform t;
    tf::poseMsgToTF(approach_pose.pose, t);
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "approach_pose"));

    return move_group_util::set_pose_target_and_execute(move_group, approach_pose.pose, 0);
}

bool CaddyGrasping::lift_up(bool plan_only, double lift_offset) {
    auto pose_target = grasp_pose;
    pose_target.pose.position.z += lift_offset;

    return move_group_util::set_pose_target_and_execute(move_group, pose_target.pose, 0);
}

bool CaddyGrasping::grasp(bool plan_only) {

    bool success = move_group_util::set_pose_target_and_execute(move_group, grasp_pose.pose, 0);
    if (! plan_only) {
        gripper.close();
    }
    return success;
}