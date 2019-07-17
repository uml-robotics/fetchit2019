#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

class HandleDetector {

public:
    HandleDetector() : no_handle_cloud(new pcl::PointCloud<pcl::PointXYZ>){}

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_no_handle_cloud(){
        return no_handle_cloud;
    }

    tf::StampedTransform detect_from_scene_cloud(const std::string &point_cloud_topic);

private:
    tf::TransformBroadcaster broadcaster;

    tf::StampedTransform detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_handle_cloud;
};
