#include <ros/ros.h>
#include "NavOnlyArena.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_nav");
    ros::NodeHandle nh;


    Arena* arena = new NavOnlyArena();
    arena->run();

    return 0;
}