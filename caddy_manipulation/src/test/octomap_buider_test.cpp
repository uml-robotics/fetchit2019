#include <ros/ros.h>
#include "caddy_manipulation/OctomapBuilder.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_caddy_test");
    ros::NodeHandle nh;

    OctomapBuilder octomap_builder(nh);
    octomap_builder.clear_map();
    octomap_builder.look_around_to_build_map();

    std::cout << "Press Enter to clear octomap";
    std::cin.ignore();

    octomap_builder.clear_map();

    std::cout << "Press Enter to restore octomap";
    std::cin.ignore();

    octomap_builder.restore_map();

    ros::Duration(1).sleep();

    return 0;
}
