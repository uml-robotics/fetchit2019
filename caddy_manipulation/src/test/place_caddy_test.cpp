#include <ros/ros.h>
#include "caddy_manipulation/PlacingCaddy.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "place_caddy_test");

    PlacingCaddy();

    return 0;
}