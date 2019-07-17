#include "kit_detector/transformPointCloud.h"

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

tf::StampedTransform lookup_transform(const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;

    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform (target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return transform;
}

void transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2 &in,
                         pcl::PointCloud<pcl::PointXYZ> &out) {
    sensor_msgs::PointCloud2::Ptr transformed_cloud(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(target_frame, lookup_transform(target_frame, in.header.frame_id), in, *transformed_cloud);

    pcl::fromROSMsg(*transformed_cloud, out);
}

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointXYZ> &in,
                         pcl::PointCloud<pcl::PointXYZ> &out) {
    ROS_INFO_STREAM("Transforming PointCloud from " << in.header.frame_id << " to " << target_frame);
    pcl_ros::transformPointCloud(in, out, lookup_transform(target_frame, in.header.frame_id));
}

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointNormal> &in, pcl::PointCloud<pcl::PointNormal> &out) {
    ROS_INFO_STREAM("Transforming PointCloud from " << in.header.frame_id << " to " << target_frame);
    pcl_ros::transformPointCloud(in, out, lookup_transform(target_frame, in.header.frame_id));
}