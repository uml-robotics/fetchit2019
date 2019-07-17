#include "schunk_door_detector/HandlePoseEstimator.h"

#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/pca.h>
#include "common_perception/crop.h"
#include <pcl/common/common.h>
#include <angles/angles.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/extract_indices.h>

pcl::PointXYZ get_highest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud, indices));
    int i;
    subcloud->getMatrixXfMap().row(2).maxCoeff(&i);
    return subcloud->points[i];
}

HandlePoseEstimator::HandlePoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &handles) : no_handle_cloud(new pcl::PointCloud<pcl::PointXYZ>){
//Remove wall
//    pcl::io::savePCDFile("wall.pcd", *handles);
    pcl::PointXYZ furthest_point;
    int furthest_point_index;
    handles->getMatrixXfMap().row(0).maxCoeff(&furthest_point_index);
    furthest_point = handles->points[furthest_point_index];

    ROS_INFO_STREAM("FURTHEST POINT X: " << furthest_point.x);

    pcl::IndicesPtr furthest_indices(new std::vector<int>);
    pcl::IndicesPtr closest_indices(new std::vector<int>);
    pcl::IndicesPtr height_indices(new std::vector<int>);
    crop_x<pcl::PointXYZ>(handles, 1.05, 10, *furthest_indices);
    crop_x<pcl::PointXYZ>(handles, 0, 0.5, *closest_indices);
    furthest_indices->insert(furthest_indices->end(), closest_indices->begin(), closest_indices->end());
    crop_z<pcl::PointXYZ>(handles, 1.3, 10, *height_indices);
    furthest_indices->insert(furthest_indices->end(), height_indices->begin(), height_indices->end());

    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(handles);
    ex.setIndices(furthest_indices);
    ex.setNegative(true);
    ex.filter(*handles);
    //pcl::io::savePCDFile("no_wall.pcd", *handles);
//End of remove wall

    // get highest_point
    {
        int highest_point_index;
        handles->getMatrixXfMap().row(2).maxCoeff(&highest_point_index);
        highest_point = handles->points[highest_point_index];
    }

    static tf::TransformBroadcaster br;

    // crop handle
    pcl::IndicesPtr handles_indices(new std::vector<int>);
    //ROS_INFO_STREAM("HIGHEST POINT Z: " << highest_point.z);
    crop_z<pcl::PointXYZ>(handles, highest_point.z - 0.032, highest_point.z, *handles_indices);
    //pcl::io::savePCDFile("handle.pcd", *handles, *handles_indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(handles);
    extract.setIndices(handles_indices);
    extract.setNegative(true);
    extract.filter(*no_handle_cloud);

    //pcl::io::savePCDFile("no_handle.pcd", *no_handle_cloud);


    std::vector<pcl::PointIndices> clusters_indices;
    cluster_handles(handles, handles_indices, clusters_indices);
    ROS_INFO_STREAM("clustering...");


    for (int i = 0; i <clusters_indices.size(); i++) {

        auto &handle_indices = clusters_indices[i];

        Eigen::RowVector3f major_vector = {1, 0, 0};

        // some ugly conversions...
        Eigen::Quaternionf eigen_quaternionf = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitY(), major_vector);
        tf::Quaternion tf_quaternion;
        tf::quaternionEigenToTF(eigen_quaternionf.cast<double>(), tf_quaternion);
        auto sub_highest_point = get_highest_point(handles, handle_indices.indices);
        tf::Transform handle_frame(tf_quaternion, tf::Vector3(sub_highest_point.x, sub_highest_point.y, (sub_highest_point.z - 0.0381 + 0.066675)));
        br.sendTransform(tf::StampedTransform(handle_frame, ros::Time::now(), "base_link","door_handle"));
        ts = handle_frame;
    }
}

void HandlePoseEstimator::cluster_handles(pcl::PointCloud<pcl::PointXYZ>::Ptr &handle, pcl::IndicesPtr &handle_indices, std::vector<pcl::PointIndices> &cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(handle, handle_indices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(handle);
    ec.setIndices(handle_indices);
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(20); // heuristic
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
    ROS_INFO_STREAM(cluster_indices.size() << " clusters. sorting from left to right...");

    // sort from left to right

    std::sort(cluster_indices.begin(), cluster_indices.end(), [&handle](pcl::PointIndices a, pcl::PointIndices b) {
        pcl::PointXYZ a_centroid; pcl::computeCentroid(*handle, a.indices, a_centroid);
        pcl::PointXYZ b_centroid; pcl::computeCentroid(*handle, b.indices, b_centroid);
        return a_centroid.y > b_centroid.y;
    });

}
