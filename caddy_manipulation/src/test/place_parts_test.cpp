#include <ros/ros.h>
#include <caddy_manipulation/PlacingParts.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "place_parts_test");

    PlacingParts p;
//    p.place_bolt();
//    p.place_gearbox();
    p.place_large_gear();
//    p.place_small_gear();
}