#include "common_perception/TableSegmentor.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <ros/console.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/point_cloud.h>

TableSegmentor::TableSegmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::IndicesPtr &indices) :
    in(input),
    indices(indices),
    coefficients(new pcl::ModelCoefficients),
    inliers(new pcl::PointIndices),
    table_hull_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    ros::NodeHandle nh;
    pub_hull = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/table_hull", 0);
}

void TableSegmentor::segment(double eps_degree, double distance_threshold) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); // z axis
    seg.setEpsAngle(pcl::deg2rad(eps_degree)); // allowed error
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(in);
    seg.setIndices(indices);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_ERROR("Could not estimate a planar model from the table area.");
    }
    else {
        ROS_DEBUG_STREAM(inliers->indices.size() << " inliers");
    }

    build_table_hull_cloud();
}

void TableSegmentor::build_table_hull_cloud() {
    // Project the model inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(in);
    proj.setIndices(inliers);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // Create a Concave Hull representation of the projected inliers
    pcl::ConcaveHull<pcl::PointXYZ> cHull;
    cHull.setDimension(2);
    cHull.setInputCloud(cloud_projected);
    cHull.setAlpha(0.1);
    cHull.reconstruct(*table_hull_cloud);
    pub_hull.publish(table_hull_cloud);

    // set min max points
    pcl::getMinMax3D (*table_hull_cloud, minPt, maxPt);
    table_height = (minPt[2] + maxPt[2]) / 2.0f;
    minPt[2] = table_height; // min z
    // add some allowance (for objects on edge)
    minPt[0] -= 0.05;
    maxPt[0] -= 0.02;

}

pcl::IndicesPtr TableSegmentor::getNonTableIndices() {
    pcl::IndicesPtr ret(new std::vector<int>);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*ret);

    return ret;
}

void TableSegmentor::getAboveTable(float max, std::vector<int> &indices_out) {
    maxPt[2] = max; // max z

    ROS_DEBUG_STREAM("cropping above table:");
    ROS_DEBUG_STREAM("min: \n" << minPt);
    ROS_DEBUG_STREAM("max: \n" << maxPt);

    // crop box
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(minPt);
    box_filter.setMax(maxPt);
    box_filter.setInputCloud(in);
    box_filter.setIndices(getNonTableIndices());
    box_filter.filter(indices_out);
}

