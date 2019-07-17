#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <fetch_cpp/HeadClient.h>
#include <fetch_cpp/PointHeadClient.h>

class OctomapBuilder {

public:
    OctomapBuilder(ros::NodeHandle &nh) : nh(nh) {
        points_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("rh/points_for_moveit_octomap", 1);
    }

    void clear_map() {

        ROS_INFO_STREAM("wait service /clear_octomap...");
        ros::service::waitForService("/clear_octomap");
        ROS_INFO_STREAM("... done");
        ros::ServiceClient clear_octomap_srv = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
        std_srvs::Empty srv;
        auto result = clear_octomap_srv.call(srv);
        ROS_INFO_STREAM("calling /clear_octomap...");
        ROS_INFO_STREAM(" " << std::boolalpha << result);
    }

    void look_around_to_build_map() {
        points_built_on.clear();

        HeadClient head;

        ROS_INFO_STREAM("pan 0");
        head.pan_head(0);
        process_and_publish();

//        ROS_INFO_STREAM("pan -45");
//        head.pan_head(-45);
//        process_and_publish();
//
//        head.pan_head(45);
//        ROS_INFO_STREAM("pan 45");
//        process_and_publish();
//
//        head.pan_head(0);
    }

    void relatively_look_around_to_build_map(std::vector<Eigen::Vector3d> poses) {
        points_built_on.clear();

        PointHeadClient point_head;

        for (auto &pose : poses) {
            point_head.relatively_look_at(pose[0], pose[1], pose[2]);
            process_and_publish();
        }
    }

    void look_up_down_to_build_map(std::vector<geometry_msgs::PoseStamped> poses) {
        points_built_on.clear();

        PointHeadClient point_head;

        for (auto &pose : poses) {
            point_head.look_at(pose);
            process_and_publish();
        }
    }

    void restore_map() {
        for (auto cloud : points_built_on)
            publish(cloud);
    }

    void process_and_publish() {
        auto points = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/head_camera/depth_downsample/points", nh);
        ROS_INFO_STREAM("raw: " << points->size() << " points");

        pcl::PointCloud<pcl::PointXYZ>::Ptr no_outlier(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(points);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*no_outlier);
        ROS_INFO_STREAM("after cleaned: " << no_outlier->size() << " points");

        publish(no_outlier);
        add_points_built_on(no_outlier);
    }

    //If you call this directly then the restore function will not work
    void publish(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &points) {
        points_pub.publish(points);
        ros::spinOnce();
    }

private:
    ros::NodeHandle &nh;

    tf::TransformListener tf_listener;

    ros::Publisher points_pub;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_built_on;



    void add_points_built_on(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) {
//        ROS_DEBUG_STREAM("adding points...");
//        pcl::PointCloud<pcl::PointXYZ>::Ptr points_at_base_link(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl_ros::transformPointCloud("base_link", *points, *points_at_base_link, tf_listener);
//        *points_built_on += *points_at_base_link;

        points_built_on.emplace_back(points);
    }

};
