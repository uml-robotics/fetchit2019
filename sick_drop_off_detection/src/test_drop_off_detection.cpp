#include <ros/ros.h>
#include "sick_drop_off_detection/SickDropOffDetector.h"
#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include <fetch_cpp/HeadClient.h>

int main(int argc, char **argv) {
ROS_INFO_STREAM("STARTING");
    ros::init(argc, argv, "drop_off_detector");
    ros::NodeHandle nh;

    TorsoLiftClient torsoLiftClient;
    torsoLiftClient.lift_to_highest();

    PointHeadClient headClient;
    headClient.relatively_look_at(0.3, 0, 0);

    SickDropOffDetector d;

    auto topic = "/head_camera/depth_downsample/points";

//    ros::Rate r(6);
//    while(ros::ok()) {
        d.detect_from_scene_cloud(topic);
//        ros::spinOnce();
//        r.sleep();
//    }

    return 0;
}
