#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/PointIndices.h>

class ChuckDetector {
public:
    ChuckDetector();

    geometry_msgs::PoseStamped detect_chuck(const std::string &topic = "/head_camera/depth_registered/points");
    geometry_msgs::PoseStamped detect_chuck(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

private:
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    ros::Publisher pose_pub;

    inline void visualize (const pcl::PointCloud<pcl::PointXYZ> &pc, const std::vector<int> &indices);

    geometry_msgs::PoseStamped detect_chuck_rel_to_base_link(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    geometry_msgs::PoseStamped detect_chuck_from_machine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &indices);

    void throw_bad_argument_if_empty_cluster(std::vector<int> &_) {
        if (_.empty()) {
            throw std::invalid_argument("empty cluster");
        }
    }
};