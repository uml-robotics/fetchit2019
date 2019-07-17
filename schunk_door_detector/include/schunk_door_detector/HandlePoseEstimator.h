#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/LinearMath/Transform.h>
#include <pcl/visualization/pcl_visualizer.h>

class HandlePoseEstimator {

public:
    explicit HandlePoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &handles);

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_no_handle_cloud(){
	return no_handle_cloud;
    }


    tf::Transform& get_transfroms() {
        return ts;
    }

private:
    tf::Transform ts;

    pcl::PointXYZ highest_point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_handle_cloud;

    void cluster_handles(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddy, pcl::IndicesPtr &handle_indices, std::vector<pcl::PointIndices> &cluster_indices);
};

