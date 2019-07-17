#include "sick_drop_off_detection/SickDropOffPoseEstimator.h"

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

SickDropOffPoseEstimator::SickDropOffPoseEstimator(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
//TODO:Get corner
    int corner_point_index;
    cloud->getMatrixXfMap().row(0).minCoeff(&corner_point_index);
    max_point_x = cloud->points[corner_point_index];
    cloud->getMatrixXfMap().row(1).minCoeff(&corner_point_index);
    max_point_y = cloud->points[corner_point_index];

    //static tf::TransformBroadcaster br;

    Eigen::RowVector3f major_vector = {1, 0, 0};

        // some ugly conversions...
    Eigen::Quaternionf eigen_quaternionf = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitY(), major_vector);
    tf::Quaternion tf_quaternion;
    tf::quaternionEigenToTF(eigen_quaternionf.cast<double>(), tf_quaternion);
    tf::Transform cloud_frame(tf_quaternion, tf::Vector3(max_point_x.x, max_point_y.y, (max_point_y.z + max_point_x.z)/2));
    //br.sendTransform(tf::StampedTransform(cloud_frame, ros::Time::now(), "base_link","corner"));
    ts = cloud_frame;
}
