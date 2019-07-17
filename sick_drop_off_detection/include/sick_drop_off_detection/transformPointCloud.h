#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

void transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2 &in,
                         pcl::PointCloud<pcl::PointXYZ> &out);

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointXYZ> &in,
                         pcl::PointCloud<pcl::PointXYZ> &out);

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointNormal> &in, pcl::PointCloud<pcl::PointNormal> &out);


