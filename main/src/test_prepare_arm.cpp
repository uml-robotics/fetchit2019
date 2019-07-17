#include <ros/ros.h>
#include "TestCourse.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_prepare_arm");

    Arena* arena = new TestCourse(1, false);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    MoveFetch move_fetch;
    arena->prepare_to_pick(move_fetch);

    return 0;
}