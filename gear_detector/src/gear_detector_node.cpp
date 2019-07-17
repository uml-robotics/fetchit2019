#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <common_perception/TableSegmentor.h>
#include <common_perception/crop.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/io/pcd_io.h>
//using namespace pcl::io;

typedef pcl::PointCloud<pcl::PointXYZ> PC;

#include <pcl/filters/statistical_outlier_removal.h>
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

std::vector<int> extract_above_table(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::vector<int> above_table_indices;

    // crop table area
    pcl::IndicesPtr z_indices(new std::vector<int>);
    crop_z<pcl::PointXYZ>(cloud, 0.5, 1.5, *z_indices); // 0.5 to remove ground, 1.5 is the height of the robot
    pcl::IndicesPtr z_x_indices(new std::vector<int>);
    crop_x<pcl::PointXYZ>(cloud, z_indices, 0, 2, *z_x_indices);

    // segment plane
    TableSegmentor tableSegmentor(cloud, z_x_indices);
    tableSegmentor.segment();

    // get above table
    tableSegmentor.getAboveTable(100, above_table_indices);

    return above_table_indices;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gear_detector");

    ros::NodeHandle nh;

    tf::TransformListener tf_listener;

    auto topic = "/head_camera/depth_downsample/points";
//    auto topic = "/head_camera/depth_registered/points";

    ROS_INFO_STREAM("waiting for " << topic);
    auto raw_scene = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>(topic, nh);

    // transform to base_link
    auto raw_scene_at_base = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    tf_listener.waitForTransform(raw_scene->header.frame_id, "base_link", ros::Time::now(), ros::Duration(5.0));
    pcl_ros::transformPointCloud("base_link", *raw_scene, *raw_scene_at_base, tf_listener);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = clean_raw_scene(raw_scene_at_base);

    // get above
    std::vector<int> above_table_indices = extract_above_table(scene);
    pcl::io::savePCDFile("above_table.pcd", *scene, above_table_indices);
    pcl::IndicesConstPtr above_table_indices_ptr = boost::make_shared<std::vector<int>>(above_table_indices); // ugly!

    // cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(scene, above_table_indices_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(scene);
    ec.setIndices(above_table_indices_ptr);
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(20); // heuristic
    ec.setMaxClusterSize(800); // exclude machine part
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    const auto n_clusters = cluster_indices.size();
    ROS_INFO_STREAM(n_clusters << " clusters");

//    for (int i = 0; i < cluster_indices.size(); ++i) {
//        savePCDFile("cluster_" + std::to_string(i) + ".pcd", PC(*scene, cluster_indices.at(i).indices));
//    }
}