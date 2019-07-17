#include "kit_detector/KitDetector.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include <pcl/filters/filter.h>

#include "kit_detector/transformPointCloud.h"
#include <common_perception/TableSegmentor.h>
#include "common_perception/crop.h"
#include "kit_detector/CaddyPoseEstimator.h"

#include "kit_detector/Caddy.h"

#include "pcl/io/pcd_io.h"

#include <stdexcept>
#include <pcl/common/common.h>

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

std::vector<KitPoses> KitDetector::detect_from_scene_cloud(const std::string &topic) {
    std::vector<KitPoses> results;

    bool corrupted = true; // assume corrupted

    ros::NodeHandle nh;
    sensor_msgs::PointCloud2::ConstPtr msg;

    while (corrupted) {
        ROS_INFO_STREAM("Waiting for " << topic);
        msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
        try {
            results = detect_from_scene_cloud(msg);
            corrupted = false;
        }
        catch (std::invalid_argument& e) {
            ROS_DEBUG_STREAM(e.what());
            ROS_DEBUG_STREAM("corrupted msg - retrying...");
            corrupted = true;
        }
    }

    return results;
}
std::vector<KitPoses> KitDetector::detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromROSMsg(*msg, *temp_cloud);
//    pcl::io::savePCDFile("raw_scene.pcd", *temp_cloud);

    std::vector<KitPoses> poses;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_scene(new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloud("base_link", *msg, *raw_scene);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = clean_raw_scene(raw_scene);
    pcl::io::savePCDFile("kd_clean_scene.pcd", *scene);

    pcl::IndicesPtr above_table_indices(new std::vector<int>);
    extract_above_table(scene, *above_table_indices);
    pcl::io::savePCDFile("kd_above_table.pcd", *scene, *above_table_indices);

    auto above_table_base_link = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*scene, *above_table_indices));
    if (above_table_base_link->width == 0) {
        throw std::invalid_argument("above_table_base_link with is 0.");
    }
    CaddyPoseEstimator e(above_table_base_link);

    const auto handle_center_t = tf::Vector3(0, 0, CaddyModel::top_z - Caddy::handle_bar_height / 2.0);
    // To get the position changes, import .stl in Blender, select and subdivide corresponding faces, select the vertex
    const auto gearbox_compartment_t = tf::Vector3(0.058737, 0, 0);
    const auto bolt_compartment_t  = tf::Vector3(-0.061237,  0.056238, 0);
    const auto gears_compartment_t = tf::Vector3(-0.061237, -0.056238, 0);
    for (int i = 0; i < e.get_transfroms().size(); ++i) {
        std::string caddy_name = "caddy_" + std::to_string(i);
        auto transform = tf::StampedTransform(e.get_transfroms()[i], ros::Time::now(), "base_link", caddy_name);
        broadcaster.sendTransform(transform);

        poses.push_back({
            transform,
            translate_caddy_pose(transform, caddy_name + "_handle_center", handle_center_t),
            translate_caddy_pose(transform, caddy_name + "_gearbox_compartment", gearbox_compartment_t),
            translate_caddy_pose(transform, caddy_name + "_bolt_compartment", bolt_compartment_t),
            translate_caddy_pose(transform, caddy_name + "_gears_compartment", gears_compartment_t)
        });
    }

    return poses;
}

geometry_msgs::PoseStamped KitDetector::translate_caddy_pose(tf::StampedTransform &transform, const std::string &child_frame, const tf::Vector3 &translation) {
    geometry_msgs::PoseStamped ret;
    tf::StampedTransform st(transform * tf::Transform(tf::Quaternion(0, 0, 0, 1), translation), ros::Time::now(), "base_link", child_frame);
    broadcaster.sendTransform(st);

    ret.header.frame_id = "base_link";
    tf::poseTFToMsg(st, ret.pose);

    return ret;
}

void KitDetector::extract_above_table(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &above_table_indices) {
    // crop table area
    pcl::IndicesPtr cropped(new std::vector<int>);
    Eigen::Vector4f min_pt(0.5, -0.7, 0.5, 1), max_pt(0.9, 0.7, 1, 1);
    pcl::getPointsInBox(*cloud, min_pt, max_pt, *cropped);

    if (cropped->empty()) {
        throw std::invalid_argument("Table area cloud is empty.");
    }

    ROS_DEBUG_STREAM("table area: " << cropped->size() << " points");
    pcl::io::savePCDFile("kd_table_area.pcd", *cloud, *cropped);

    // segment plane
    TableSegmentor tableSegmentor(cloud, cropped);
    tableSegmentor.segment();

    // get above table
    tableSegmentor.getAboveTable(2, above_table_indices);
}
