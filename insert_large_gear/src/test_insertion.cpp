#include <ros/ros.h>
#include "insert_large_gear/LargeGearInserting.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_insert_large_gear");

    LargeGearInserting i;
    i.insert();
//    i.remove();

    return 0;
}