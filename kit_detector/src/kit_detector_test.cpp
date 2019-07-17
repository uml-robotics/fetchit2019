#include <ros/ros.h>
#include "kit_detector/KitDetector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "kit_detector");
    KitDetector kd;

    ros::Rate r(15);
    while(ros::ok()) {
        kd.detect_from_scene_cloud("/head_camera/depth_downsample/points");
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
