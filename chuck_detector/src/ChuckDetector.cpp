#include "chuck_detector/ChuckDetector.h"

#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <common_perception/TableSegmentor.h>
#include <common_perception/crop.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/io/pcd_io.h>
using namespace pcl::io;

ChuckDetector::ChuckDetector() {
    cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("chuck_detector", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("chuck_pose", 1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr clean_raw_scene(pcl::PointCloud<pcl::PointXYZ>::ConstPtr &raw_scene) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> _;
    pcl::removeNaNFromPointCloud(*raw_scene, *scene_no_nan, _);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(scene_no_nan);
    sor.setMeanK(100);
    sor.setStddevMulThresh(2.0);
    sor.filter(*scene);

    return scene;
}

geometry_msgs::PoseStamped ChuckDetector::detect_chuck(const std::string &topic) {
    ROS_INFO_STREAM("waiting for " << topic);
    auto raw_scene = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>(topic, nh);
    ROS_INFO_STREAM("received for " << topic);

    geometry_msgs::PoseStamped result;
    try {
        savePCDFile("raw_scene.pcd", *raw_scene);
        auto cleaned_scene = clean_raw_scene(raw_scene);

        return detect_chuck(cleaned_scene);
    }
    catch(pcl::IOException &_) {
        throw std::invalid_argument("pcl io exception");
    }

    return result;
}

geometry_msgs::PoseStamped ChuckDetector::detect_chuck(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    tf::TransformListener tf_listener;

    tf_listener.waitForTransform("base_link", cloud->header.frame_id, ros::Time(0), ros::Duration(5));
    tf::StampedTransform to_base_link_transform;
    tf_listener.lookupTransform("base_link", cloud->header.frame_id, ros::Time(0), to_base_link_transform);


    pcl::PointCloud<pcl::PointXYZ>::Ptr base_link_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud, *base_link_cloud, to_base_link_transform);
    savePCDFile("base_link_cloud.pcd", *base_link_cloud);

    return detect_chuck_rel_to_base_link(base_link_cloud);
}

std::vector<int> extract_above_table(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::vector<int> above_table_indices;

    // crop table area
    pcl::IndicesPtr z_indices(new std::vector<int>);
    crop_z<pcl::PointXYZ>(cloud, 0.5, 2, *z_indices); // 0.5 to remove ground
    pcl::IndicesPtr z_x_indices(new std::vector<int>);
    crop_x<pcl::PointXYZ>(cloud, z_indices, 0, 2, *z_x_indices);

    // segment plane
    TableSegmentor tableSegmentor(cloud, z_x_indices);
    tableSegmentor.segment(8);

    // get above table
    tableSegmentor.getAboveTable(100, above_table_indices);

    return above_table_indices;
}

geometry_msgs::PoseStamped ChuckDetector::detect_chuck_rel_to_base_link(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    std::vector<int> above_table_indices = extract_above_table(cloud);
    pcl::io::savePCDFile("above_table.pcd", *cloud, above_table_indices);


    pcl::IndicesConstPtr above_table_indices_ptr = boost::make_shared<std::vector<int>>(above_table_indices); // ugly!

    ROS_INFO_STREAM("cluster & get machine cluster...");
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud, above_table_indices_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud);
    ec.setIndices(above_table_indices_ptr);
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    const auto n_clusters = cluster_indices.size();
    ROS_INFO_STREAM(n_clusters << " clusters");

    auto largest_cluster = std::max_element(cluster_indices.begin(), cluster_indices.end(),
                                            [](pcl::PointIndices const l, pcl::PointIndices const r) {
                                                return l.indices.size() < r.indices.size();
                                            }
    );

    savePCDFile("largest_cluster.pcd", pcl::PointCloud<pcl::PointXYZ>(*cloud, largest_cluster->indices));
    for (int i = 0; i < cluster_indices.size(); ++i) {
        savePCDFile("cluster_" + std::to_string(i) + ".pcd",
                    pcl::PointCloud<pcl::PointXYZ>(*cloud, cluster_indices.at(i).indices));
    }

    return detect_chuck_from_machine(cloud, largest_cluster->indices);
}

geometry_msgs::PoseStamped ChuckDetector::detect_chuck_from_machine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &indices) {

    ROS_INFO_STREAM("detecting bottom plane...");
    pcl::PointIndices bottom_plane_indices;
    {
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f(0, 0, 1)); // z axis
        seg.setEpsAngle(pcl::deg2rad(10.0));
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.01);

        // input & indices
        seg.setInputCloud(cloud);
        pcl::IndicesConstPtr indices_ptr = boost::make_shared<std::vector<int>>(indices); // ugly!
        seg.setIndices(indices_ptr);

        seg.segment(inliers, coefficients);

        if (inliers.indices.empty()) {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }
        ROS_DEBUG_STREAM("Model coefficients: " << coefficients);
        ROS_DEBUG_STREAM("Model inliers: " << inliers.indices.size() << " points" << " - " << inliers.header);

        savePCDFile("bottom_plane.pcd", *cloud, inliers.indices);

        bottom_plane_indices = inliers;
    }

    ROS_INFO_STREAM("cluster & get largest in bottom plane (contains edge)...");
    std::vector<int> largest_cluster_of_bottom_plane;
    {
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::IndicesConstPtr bottom_plane_indices_ptr = boost::make_shared<std::vector<int>>(bottom_plane_indices.indices); // ugly!
        tree->setInputCloud(cloud, bottom_plane_indices_ptr);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud);
        ec.setIndices(bottom_plane_indices_ptr);
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.extract(cluster_indices);

        const auto n_clusters = cluster_indices.size();
        ROS_INFO_STREAM(n_clusters << " clusters");

        if (n_clusters == 0) {
            throw std::invalid_argument("0 clusters");
        }

        auto largest_cluster = std::max_element(cluster_indices.begin(), cluster_indices.end(),
                                                [](pcl::PointIndices const l, pcl::PointIndices const r) {
                                                    return l.indices.size() < r.indices.size();
                                                }
        );

        throw_bad_argument_if_empty_cluster(largest_cluster->indices);

        savePCDFile("bottom_plane_largest_cluster.pcd",
                    pcl::PointCloud<pcl::PointXYZ>(*cloud, largest_cluster->indices));
        for (int i = 0; i < cluster_indices.size(); ++i) {
            throw_bad_argument_if_empty_cluster(cluster_indices.at(i).indices);
            savePCDFile("bottom_plane_cluster_" + std::to_string(i) + ".pcd", pcl::PointCloud<pcl::PointXYZ>(*cloud, cluster_indices.at(i).indices));
        }
        largest_cluster_of_bottom_plane = largest_cluster->indices;
    }

    std::vector<int> above_bottom_plane;
    {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, largest_cluster_of_bottom_plane, min_pt, max_pt);
        // x
        max_pt[0] -= 0.05;
        // y
        min_pt[1] += 0.1;
        max_pt[1] -= 0.1;
        // z
        min_pt[2] += 0.05;
        max_pt[2] = 1.1;
        pcl::getPointsInBox(*cloud, min_pt, max_pt, above_bottom_plane);
        throw_bad_argument_if_empty_cluster(above_bottom_plane);
        savePCDFile("above_bottom_panel.pcd", pcl::PointCloud<pcl::PointXYZ>(*cloud, above_bottom_plane));
    }

    std::vector<int> above_bottom_plane_cropped;
    {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, above_bottom_plane, min_pt, max_pt);
        max_pt[1] = min_pt[1] + 4; // y max relative to min
        pcl::getPointsInBox(*cloud, min_pt, max_pt, above_bottom_plane_cropped);
        throw_bad_argument_if_empty_cluster(above_bottom_plane_cropped);
        savePCDFile("above_bottom_plane_cropped.pcd", pcl::PointCloud<pcl::PointXYZ>(*cloud, above_bottom_plane_cropped));
    }

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, above_bottom_plane_cropped, min_pt, max_pt);
    auto z = (min_pt[2] + max_pt[2]) / 2.0;
    auto y = min_pt[1];
    auto x = min_pt[0] + 0.065;

    geometry_msgs::PoseStamped chuck_pose_at_base_link;
    chuck_pose_at_base_link.header.stamp = ros::Time(0);
    chuck_pose_at_base_link.header.frame_id = "base_link";
    chuck_pose_at_base_link.pose.position.x = x;
    chuck_pose_at_base_link.pose.position.y = y;
    chuck_pose_at_base_link.pose.position.z = z;
    chuck_pose_at_base_link.pose.orientation.x = 0;
    chuck_pose_at_base_link.pose.orientation.y = 0;
    chuck_pose_at_base_link.pose.orientation.z = 0;
    chuck_pose_at_base_link.pose.orientation.w = 1;

    pose_pub.publish(chuck_pose_at_base_link);
    ros::spinOnce();

    return chuck_pose_at_base_link;
}

void ChuckDetector::visualize(const pcl::PointCloud<pcl::PointXYZ> &pc, const std::vector<int> &indices) {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inliers_cloud(new pcl::PointCloud<pcl::PointXYZ>(pc, indices));
    cloud_pub.publish(inliers_cloud);
}