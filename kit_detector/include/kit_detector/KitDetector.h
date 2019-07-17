#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

struct KitPoses {
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped handle_center;
    geometry_msgs::PoseStamped gearbox_compartment, bolt_compartment, gears_compartment;
};

class KitDetector {

public:
    KitDetector() = default;

    std::vector<KitPoses> detect_from_scene_cloud(const std::string &point_cloud_topic);


private:

    tf::TransformBroadcaster broadcaster;

    std::vector<KitPoses> detect_from_scene_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void static extract_above_table(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<int> &above_table_indices);

    geometry_msgs::PoseStamped translate_caddy_pose(tf::StampedTransform &transform, const std::string &child_frame, const tf::Vector3 &translation);
};
