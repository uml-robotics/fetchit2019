#include "sick_drop_off_detection/SickDropOffDetector.h"
#include "sick_drop_off_detection/SickDropOffPoseEstimator.h"
#include "sick_drop_off_detection/transformPointCloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>
#include <pcl/filters/filter.h>
#include <common_perception/TableSegmentor.h>
#include <common_perception/crop.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr clean_raw_scene(pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_scene) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> _;
    pcl::removeNaNFromPointCloud(*raw_scene, *scene_no_nan, _);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(scene_no_nan);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*scene);

    return scene;
}

tf::StampedTransform SickDropOffDetector::detect_from_scene_cloud(const std::string &topic) {
    tf::StampedTransform result;

    bool corrupted = true; // assume corrupted

    ros::NodeHandle nh;
    sensor_msgs::PointCloud2::ConstPtr msg;

    while (corrupted) {
        ROS_INFO_STREAM("Waiting for " << topic);
        msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
        try {
            result = detect_from_scene_cloud(msg);
            corrupted = false;
        }
        catch (std::invalid_argument& e) {
            ROS_DEBUG_STREAM(e.what());
            ROS_DEBUG_STREAM("corrupted msg - retrying...");
            corrupted = true;
        }
    }

    return result;
}

tf::StampedTransform SickDropOffDetector::detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    //pcl::io::savePCDFile("raw_scene.pcd", *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_scene(new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloud("base_link", *msg, *raw_scene);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = clean_raw_scene(raw_scene);
    //pcl::io::savePCDFile("clean_scene.pcd", *scene);

    pcl::IndicesPtr z_indices(new std::vector<int>);
    crop_z<pcl::PointXYZ>(scene, 0.5, 1.5, *z_indices); // 0.5 to remove ground, 1.5 is the height of the robot
    pcl::IndicesPtr z_x_indices(new std::vector<int>);
    crop_x<pcl::PointXYZ>(scene, z_indices, 0, 0.75, *z_x_indices);
    pcl::IndicesPtr z_x_y_indices(new std::vector<int>);
    crop_y<pcl::PointXYZ>(scene, z_x_indices, -0.75, 0, *z_x_y_indices);

    // Get Table
    TableSegmentor tableSegmentor(scene, z_x_y_indices);
    tableSegmentor.segment();
    pcl::PointIndices::Ptr table_indices = tableSegmentor.getTableIndices();

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (scene);
    extract.setIndices (table_indices);
    extract.filter (*scene);

    //pcl::io::savePCDFile("table.pcd", *scene);

    SickDropOffPoseEstimator e(scene);

    auto transform = tf::StampedTransform(e.get_transfroms(), ros::Time::now(), "base_link", "drop_off_corner");
    broadcaster.sendTransform(transform);

    return transform;
}
