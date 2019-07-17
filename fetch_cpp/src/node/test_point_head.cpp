#include <ros/ros.h>
#include "fetch_cpp/PointHeadClient.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_point_head");
    ros::NodeHandle nh;

    PointHeadClient headClient;
    //headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );
    headClient.relatively_look_at(1, 0.5, -0.4);

    return 0;
}
