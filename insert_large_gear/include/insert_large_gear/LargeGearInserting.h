#pragma once

#include <tf/transform_broadcaster.h>
#include <caddy_manipulation/OctomapBuilder.h>
#include <sensor_msgs/JointState.h>

class LargeGearInserting {
public:
    LargeGearInserting();

    bool insert();

    void remove();

private:
    geometry_msgs::PoseStamped approach_retreat_pose, insert_start_pose, insert_end_pose;

    ros::NodeHandle nh;

    OctomapBuilder octomap_builder;

    tf::TransformBroadcaster broadcaster;

    geometry_msgs::PoseStamped convert_to_gripper_pose(geometry_msgs::PoseStamped &chuck_pose);

    void broadcast_frame(geometry_msgs::PoseStamped &p, const std::string &frame_id);

    bool detect();

    boost::shared_ptr<sensor_msgs::JointState const> get_valid_joint_values() {
        auto joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        while (joint_states->name.size() == 2) { // sometimes only return 2 fingers
            joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        }
        return joint_states;
    }

    bool is_inserted(boost::shared_ptr<sensor_msgs::JointState const> &before);
};