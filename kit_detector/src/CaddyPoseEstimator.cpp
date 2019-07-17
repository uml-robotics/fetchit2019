#include "kit_detector/CaddyPoseEstimator.h"

#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/pca.h>
#include "common_perception/crop.h"
#include <pcl/common/common.h>
#include <angles/angles.h>
#include "kit_detector/Caddy.h"
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/io/pcd_io.h>
#include "kit_detector/CaddyRotationParam.h"

pcl::PointXYZ get_highest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud, indices));
    int i;
    subcloud->getMatrixXfMap().row(2).maxCoeff(&i);
    return subcloud->points[i];
}

CaddyPoseEstimator::CaddyPoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddies) {
    // get highest_point
    {
        int highest_point_index;
        caddies->getMatrixXfMap().row(2).maxCoeff(&highest_point_index);
        highest_point = caddies->points[highest_point_index];
    }

    // crop handle
    pcl::IndicesPtr handles_indices(new std::vector<int>);
    crop_z<pcl::PointXYZ>(caddies, highest_point.z - Caddy::handle_bar_height * 2, highest_point.z, *handles_indices);
//    pcl::io::savePCDFile("caddy_handles.pcd", *caddies, *handles_indices);

    std::vector<pcl::PointIndices> clusters_indices;
    cluster_handles(caddies, handles_indices, clusters_indices);
    ROS_INFO_STREAM("clustering...");


    for (int i = 0; i <clusters_indices.size(); i++) {
//        if (i != 0) {
//            ROS_DEBUG_STREAM("break...");
//            break;
//        }

        auto &handle_indices = clusters_indices[i];
        // get major vector, which is the y axis of caddy
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(caddies);
        auto _indices = boost::make_shared<std::vector<int>>(handle_indices.indices);
        pca.setIndices(_indices);
        Eigen::RowVector3f major_vector = pca.getEigenVectors().col(0);

        major_vector(2) = 0; // z value is always 0, z-up
        ROS_DEBUG_STREAM("major vector: " << major_vector);

        // some ugly conversions...
        Eigen::Quaternionf eigen_quaternionf = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitY(), major_vector);
        tf::Quaternion tf_quaternion;
        tf::quaternionEigenToTF(eigen_quaternionf.cast<double>(), tf_quaternion);
        auto sub_highest_point = get_highest_point(caddies, handle_indices.indices);
        tf::Transform caddy_frame(tf_quaternion, tf::Vector3(sub_highest_point.x, sub_highest_point.y, sub_highest_point.z - CaddyModel::top_z));

        if (check_for_180_degree_rotation(caddies, caddy_frame)) {
            ROS_INFO_STREAM("ROTATING...");
            caddy_frame.setRotation(caddy_frame.getRotation() * tf::Quaternion(tf::Vector3(0, 0, 1), angles::from_degrees(180)));
        }

        // if so, remember/retrieve rotation for 1st caddy
        CaddyRotationParam rotation_store;
        if (rotation_store.is_remembering()) {
            if (i == 0) {
                ROS_INFO_STREAM("1st caddy cluster...");
                if ( ! rotation_store.was_set()) {
                    ROS_INFO_STREAM("rotation was NOT set before), setting it...");
                    ROS_INFO("%.2f %.2f %.2f %.2f", caddy_frame.getRotation().x(), caddy_frame.getRotation().y(), caddy_frame.getRotation().z(), caddy_frame.getRotation().w());
                    rotation_store.set_rotation(caddy_frame.getRotation());
                }
                else {
                    ROS_INFO_STREAM("rotation was set before, restoring it...");
                    auto retrieved = rotation_store.get_rotation();
                    ROS_INFO("%.2f %.2f %.2f %.2f", retrieved.x(), retrieved.y(), retrieved.z(), retrieved.w());
                    caddy_frame.setRotation(retrieved);
                }
            }
        }

        ts.push_back(caddy_frame);
    }
}

void CaddyPoseEstimator::cluster_handles(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddy, pcl::IndicesPtr &handle_indices, std::vector<pcl::PointIndices> &cluster_indices) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(caddy, handle_indices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(caddy);
    ec.setIndices(handle_indices);
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(20); // heuristic
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
    ROS_INFO_STREAM(cluster_indices.size() << " clusters. sorting from left to right...");

    // sort from left to right
    std::sort(cluster_indices.begin(), cluster_indices.end(), [&caddy](pcl::PointIndices a, pcl::PointIndices b) {
        pcl::PointXYZ a_centroid; pcl::computeCentroid(*caddy, a.indices, a_centroid);
        pcl::PointXYZ b_centroid; pcl::computeCentroid(*caddy, b.indices, b_centroid);
        return a_centroid.y > b_centroid.y;
    });
}

bool CaddyPoseEstimator::check_for_180_degree_rotation(pcl::PointCloud<pcl::PointXYZ>::Ptr &caddy, tf::Transform &caddy_frame) {
    ROS_DEBUG_STREAM("check_for_180_degree_rotation");
    // transform the cloud back to origin with right rotation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_back_to_origin(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*caddy, *transformed_back_to_origin, caddy_frame.inverse());
//    pcl::io::savePCDFile("transformed_back_to_origin.pcd", *transformed_back_to_origin);

    // check if left or right has more points

    auto positive_x_area = pcl::IndicesPtr(new std::vector<int>);
    Eigen::Vector4f min(0.025, -Caddy::width_4, 0.03, 0), max(0.095, Caddy::width_4, 0.06, 0);
    pcl::getPointsInBox(*transformed_back_to_origin, min, max, *positive_x_area);
    ROS_DEBUG_STREAM("[caddy] positive_x_area # of points: " << positive_x_area->size());
//    if ( ! positive_x_area->empty())
//        pcl::io::savePCDFile("positive_x_area.pcd", *transformed_back_to_origin, *positive_x_area);

    auto negative_x_area = pcl::IndicesPtr(new std::vector<int>);
    auto old_min_x = min[0];
    min[0] = -max[0];
    max[0] = -old_min_x;
    pcl::getPointsInBox(*transformed_back_to_origin, min, max, *negative_x_area);
    ROS_DEBUG_STREAM("[caddy] negative_x_area # of points: " << negative_x_area->size());
//    if ( ! negative_x_area->empty())
//        pcl::io::savePCDFile("negative_x_area.pcd", *transformed_back_to_origin, *negative_x_area);

    return positive_x_area->size() > negative_x_area->size();
}
