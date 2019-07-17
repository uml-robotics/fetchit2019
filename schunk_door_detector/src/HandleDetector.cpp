#include "schunk_door_detector/HandleDetector.hpp"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include <pcl/filters/filter.h>

#include "schunk_door_detector/transformPointCloud.h"
#include <common_perception/TableSegmentor.h>
#include "common_perception/crop.h"
#include "schunk_door_detector/HandlePoseEstimator.h"

#include "pcl/io/pcd_io.h"

#include <stdexcept>

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

tf::StampedTransform HandleDetector::detect_from_scene_cloud(const std::string &topic) {
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

tf::StampedTransform HandleDetector::detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    //pcl::io::savePCDFile("raw_scene.pcd", *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_scene(new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloud("base_link", *msg, *raw_scene);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = clean_raw_scene(raw_scene);
    //pcl::io::savePCDFile("clean_scene.pcd", *scene);

    HandlePoseEstimator e(scene);

    no_handle_cloud = e.get_no_handle_cloud();
    //pcl::io::savePCDFile("no_handle_detect.pcd", *no_handle_cloud);

    auto transform = tf::StampedTransform(e.get_transfroms(), ros::Time::now(), "base_link", "schunk_door_handle");
    broadcaster.sendTransform(transform);

    return transform;
}
