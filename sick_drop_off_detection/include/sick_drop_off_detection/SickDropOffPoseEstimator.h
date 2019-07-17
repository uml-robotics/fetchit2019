#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/LinearMath/Transform.h>
#include <pcl/visualization/pcl_visualizer.h>

class SickDropOffPoseEstimator {

public:
    explicit SickDropOffPoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    tf::Transform& get_transfroms() {
        return ts;
    }

private:
    tf::Transform ts;

    pcl::PointXYZ max_point_x;
    pcl::PointXYZ max_point_y;
};

