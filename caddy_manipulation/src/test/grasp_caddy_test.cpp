#include <ros/ros.h>
#include "caddy_manipulation/CaddyPicking.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_caddy_test");

    CaddyPicking();

    return 0;
}
