#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/LinearMath/Transform.h>
#include <pcl/visualization/pcl_visualizer.h>

class CaddyPoseEstimator {

public:
    explicit CaddyPoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddies);

    std::vector<tf::Transform>& get_transfroms() {
        return ts;
    }

private:
    std::vector<tf::Transform> ts;

    pcl::PointXYZ highest_point;

    bool static check_for_180_degree_rotation(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddy, tf::Transform &caddy_frame);

    void cluster_handles(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddy, pcl::IndicesPtr &handle_indices, std::vector<pcl::PointIndices> &cluster_indices);
};