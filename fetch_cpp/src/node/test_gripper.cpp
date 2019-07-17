#include <ros/ros.h>
#include "fetch_cpp/FetchGripper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_gripper");
    ros::NodeHandle nh;

    FetchGripper g;
    g.open();
    g.close();

    return 0;
}
