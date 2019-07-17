#include "caddy_manipulation/SickCameraClient.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_sick_camera_activate");

    SickCamera s;
    s.activate();

    return 0;
}
