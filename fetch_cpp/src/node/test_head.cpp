#include <ros/ros.h>
#include "fetch_cpp/HeadClient.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_head_client");
    ros::NodeHandle nh;

    HeadClient head;
    head.pan_head(-90);
    head.pan_head(90);
    head.pan_head(0);

    return 0;
}
