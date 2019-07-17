#include <ros/ros.h>
#include "TestCourse.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_course_node_");

    ros::NodeHandle nh("~");
    int n_th_caddy = 1;
    nh.getParam("caddy", n_th_caddy);
    ROS_INFO_STREAM("Starting with caddy " << n_th_caddy);
    ROS_INFO_STREAM("Press Enter to continue... or Ctrl-C to rerun with _caddy:=N");  std::cin.ignore();

    Arena* arena = new TestCourse(n_th_caddy);
//    arena->run();
    arena->run_forever();

    return 0;
}