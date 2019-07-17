
#include <ros/ros.h>
#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include "schunk_door_detector/HandleDetector.hpp"

int main(int argc, char **argv) {
ROS_INFO_STREAM("STARTING");
    ros::init(argc, argv, "schunk_door_detector");
    ros::NodeHandle nh;

    TorsoLiftClient torsoLiftClient;
    torsoLiftClient.lift_to_highest();

    // look at
    PointHeadClient headClient;
    headClient.look_front();
    HandleDetector hd;

    auto topic = "/head_camera/depth_downsample/points";
//    auto topic = "/head_camera/depth_registered/points";

    ros::Rate r(6);
    while(ros::ok()) {
        hd.detect_from_scene_cloud(topic);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
