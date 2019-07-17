#include <ros/ros.h>
#include "schunk_door_manip/GrabHandle.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_grab_schunk_handle");

    GrabHandle gh;
    gh.close();
    return 0;
}
