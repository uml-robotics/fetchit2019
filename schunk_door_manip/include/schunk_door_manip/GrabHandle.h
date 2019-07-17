#pragma once

#include <tf/transform_broadcaster.h>
#include <caddy_manipulation/OctomapBuilder.h>

class GrabHandle {
public:
    GrabHandle() : octomap_builder(nh), no_handle_cloud(new pcl::PointCloud<pcl::PointXYZ>){}

    void open();

    void close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_no_handle_cloud(){
        return no_handle_cloud;
    }

private:
    geometry_msgs::PoseStamped insert_start_pose, insert_end_pose, insert_handle_pose, insert_reset_pose, insert_back_pose, insert_mid_pose;

    ros::NodeHandle nh;

    OctomapBuilder octomap_builder;

    tf::TransformBroadcaster broadcaster;

    geometry_msgs::PoseStamped convert_to_gripper_pose(tf::Transform &handle_pose);

    void broadcast_frame(geometry_msgs::PoseStamped &p, const std::string &frame_id);

    void execute();

    void prepare_to_pick();

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_handle_cloud;
};
