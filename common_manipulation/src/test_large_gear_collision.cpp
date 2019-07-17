#include <ros/ros.h>
#include "common_manipulation/LargeGearCollisionObject.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_large_gear_collision");

    LargeGearCollisionObject c;
//    c.attach();
    c.detach_and_remove();

    return 0;
}