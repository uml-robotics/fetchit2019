#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>

class TableSegmentor {

public:
    float table_height; // must call after getAboveTable

    TableSegmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::IndicesPtr &indices);

    void segment(double eps_degree=10, double distance_threshold=0.01);
    pcl::PointIndices::Ptr getTableIndices() { return inliers; }
    pcl::IndicesPtr getNonTableIndices();

    // return points between table_height + allowance and max
    void getAboveTable(float max, std::vector<int> &indices_out);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr &in;
    pcl::IndicesPtr &indices;

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_hull_cloud;
    Eigen::Vector4f minPt, maxPt;

    void build_table_hull_cloud();

    // publishers
    ros::Publisher pub_hull;
};