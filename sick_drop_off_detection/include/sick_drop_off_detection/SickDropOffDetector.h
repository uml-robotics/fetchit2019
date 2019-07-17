#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

class SickDropOffDetector {

public:
    SickDropOffDetector() = default;

    tf::StampedTransform detect_from_scene_cloud(const std::string &point_cloud_topic);

private:
    tf::TransformBroadcaster broadcaster;

    tf::StampedTransform detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
};
